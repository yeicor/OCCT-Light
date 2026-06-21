// Copyright (c) 2026 Capgemini Engineering Research and Development.
//
// This file is part of OCCT-Light software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Affero General Public License version 3 as published
// by the Free Software Foundation, with an option to use any later version.
// Consult the file LICENSE_AGPL_30.txt included in OCCT-Light distribution
// for complete text of the license and disclaimer of any warranty.
//
// Alternatively, this file may be used under the terms of a commercial
// license or contractual agreement.
//
// SPDX-License-Identifier: AGPL-3.0-or-later

//! @file mesh_generate.cxx
//! @brief BRepGraphMesh_IncrementalMesh dispatch.
//!
//! Three modes folded into one entry point via the (nodes, n_nodes) pair:
//!   * 0 nodes      → whole-graph dispatch
//!   * 1 node       → single-root subtree dispatch
//!   * N>1 nodes    → multi-node dispatch
//! Bbox-derived parameter mode (use_bbox != 0) goes through the bbox
//! overload of BRepGraphMesh_IncrementalMesh.

#include "MeshCache.hxx"
#include "MeshMath.hxx"

#include "../core/ErrorState.hxx"
#include "../core/Guard.hxx"
#include "../topo/GraphHandle.hxx"
#include "../topo/TopoMath.hxx"

#include <occtl/occtl_mesh.h>

/* BRepGraphMesh_IncrementalMesh removed in OCCT 8.0.0-p1;
 * reimplemented with BRepMesh_IncrementalMesh + BRepGraph::ShapesView::Shape(). */
#include <BRepGraph_NodeId.hxx>
#include <BRepGraph_MeshView.hxx>
#include <BRepGraph_ShapesView.hxx>
#include <BRep_Builder.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <NCollection_DynamicArray.hxx>
#include <Poly_Triangulation.hxx>
#include <Precision.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>
#include <mutex>

namespace
{

//! Drops every entry in the per-graph mesh view cache. Pre-Perform
//! invalidation guarantees that a partial failure (Perform returns false
//! after touching some faces) does not leave the cache torn — every face
//! re-materialises from the current Poly_Triangulation handle on the
//! next view fetch. The per-slot version stamps in #FaceMeshBuffers /
//! #CoEdgeMeshBuffers handle the post-success path too (every mutation
//! bumps the stamp; stale slots self-evict on the next read), but a
//! single Clear() here is cheaper than walking every slot.
void invalidateCache(occtl_graph_t* const theGraph) noexcept
{
  if (theGraph == nullptr || !theGraph->meshCache)
  {
    return;
  }
  std::lock_guard<std::mutex> aLock(theGraph->meshCache->Mutex());
  theGraph->meshCache->Invalidate();
}

bool isBool01(const int32_t theValue) noexcept
{
  return theValue == 0 || theValue == 1;
}

bool IsFiniteValue(const double theValue) noexcept
{
  return !Precision::IsInfinite(theValue) && !std::isnan(theValue);
}

bool isFinitePoint(const occtl_point3_t& thePoint) noexcept
{
  return IsFiniteValue(thePoint.x) && IsFiniteValue(thePoint.y) && IsFiniteValue(thePoint.z);
}

occtl_status_t reject(const char* const theMessage)
{
  OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, theMessage);
  return OCCTL_INVALID_ARGUMENT;
}

occtl_status_t validateOptions(const occtl_mesh_options_t& theOptions)
{
  if (!OcctL::Mesh::IsKnownVersion(theOptions))
  {
    OcctL::Core::ErrorState::Current().Set(OCCTL_VERSION_MISMATCH,
                                           "unsupported mesh options struct_version");
    return OCCTL_VERSION_MISMATCH;
  }
  if (theOptions.p_next != nullptr)
  {
    return reject("mesh options p_next must be NULL");
  }
  if (!IsFiniteValue(theOptions.deflection) || theOptions.deflection <= 0.0)
  {
    return reject("mesh deflection must be finite and positive");
  }
  if (!IsFiniteValue(theOptions.angle) || theOptions.angle <= 0.0)
  {
    return reject("mesh angle must be finite and positive");
  }
  if (!IsFiniteValue(theOptions.deflection_interior) || theOptions.deflection_interior < -1.0
      || theOptions.deflection_interior == 0.0)
  {
    return reject("mesh interior deflection must be -1 or finite and positive");
  }
  if (!IsFiniteValue(theOptions.angle_interior) || theOptions.angle_interior < -1.0
      || theOptions.angle_interior == 0.0)
  {
    return reject("mesh interior angle must be -1 or finite and positive");
  }
  if (!IsFiniteValue(theOptions.min_size) || theOptions.min_size < -1.0)
  {
    return reject("mesh min_size must be -1 or finite and non-negative");
  }

  if (!isBool01(theOptions.in_parallel) || !isBool01(theOptions.relative)
      || !isBool01(theOptions.internal_vertices_mode)
      || !isBool01(theOptions.control_surface_deflection)
      || !isBool01(theOptions.control_surface_deflection_all) || !isBool01(theOptions.clean_model)
      || !isBool01(theOptions.adjust_min_size) || !isBool01(theOptions.force_face_deflection)
      || !isBool01(theOptions.allow_quality_decrease) || !isBool01(theOptions.use_bbox))
  {
    return reject("mesh boolean options must be 0 or 1");
  }

  if (!IsFiniteValue(theOptions.deviation_coefficient) || theOptions.deviation_coefficient <= 0.0)
  {
    return reject("mesh deviation_coefficient must be finite and positive");
  }
  if (!IsFiniteValue(theOptions.deviation_angle) || theOptions.deviation_angle <= 0.0)
  {
    return reject("mesh deviation_angle must be finite and positive");
  }
  if (theOptions.use_bbox != 0)
  {
    if (!isFinitePoint(theOptions.bbox.min) || !isFinitePoint(theOptions.bbox.max))
    {
      return reject("mesh bbox coordinates must be finite");
    }
    if (theOptions.bbox.min.x > theOptions.bbox.max.x
        || theOptions.bbox.min.y > theOptions.bbox.max.y
        || theOptions.bbox.min.z > theOptions.bbox.max.z)
    {
      return reject("mesh bbox min must not exceed max");
    }
  }

  return OCCTL_OK;
}

} // unnamed namespace

extern "C"
{

OCCTL_API occtl_status_t OCCTL_CALL occtl_mesh_generate(occtl_graph_t* const              graph,
                                                         const occtl_node_id_t* const      nodes,
                                                         const size_t                      n_nodes,
                                                         const occtl_mesh_options_t* const options)
{
  return OcctL::Core::Guard([&]() -> occtl_status_t {
    if (graph == nullptr || options == nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT,
                                             graph == nullptr ? "graph is NULL"
                                                              : "options is NULL");
      return OCCTL_INVALID_ARGUMENT;
    }
    if (nodes == nullptr && n_nodes != 0)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT,
                                             "nodes is NULL but n_nodes is non-zero");
      return OCCTL_INVALID_ARGUMENT;
    }

    const occtl_status_t aValidationStatus = validateOptions(*options);
    if (aValidationStatus != OCCTL_OK)
    {
      return aValidationStatus;
    }

    const IMeshTools_Parameters aParams = OcctL::Mesh::ToParameters(*options);

    BRepGraph_NodeId                           aSingleRoot;
    NCollection_LinearVector<BRepGraph_NodeId> aTargets;
    if (n_nodes == 0)
    {
      // Collect top-level root shapes.
      // 1. Root products (added via Shapes().Add()).
      const auto& aRoots = graph->graph.RootProductIds();
      for (size_t i = 0; i < aRoots.Size(); ++i)
      {
        aTargets.Append(aRoots[i]);
      }
      // 2. If there are no products (graph built directly via Editor()),
      //    fall back to top-level solids and compounds from topology.
      if (aRoots.Size() == 0)
      {
        const auto& topo = graph->graph.Topo();
        for (auto sid = topo.Solids().StartId(); sid < topo.Solids().EndId(); ++sid)
        {
          aTargets.Append(sid);
        }
        for (auto cid = topo.Compounds().StartId(); cid < topo.Compounds().EndId(); ++cid)
        {
          aTargets.Append(cid);
        }
      }
    }
    else if (n_nodes == 1)
    {
      const BRepGraph_NodeId aNodeId = OcctL::Topo::UnpackNodeId(nodes[0]);
      if (!aNodeId.IsValid())
      {
        OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_FOUND,
                                               "mesh root node is invalid or removed");
        return OCCTL_NOT_FOUND;
      }
      aTargets.Append(aNodeId);
    }
    else
    {
      for (size_t i = 0; i < n_nodes; ++i)
      {
        const BRepGraph_NodeId aId = OcctL::Topo::UnpackNodeId(nodes[i]);
        if (!aId.IsValid())
        {
          OcctL::Core::ErrorState::Current().Set(
            OCCTL_NOT_FOUND,
            "mesh node list contains an invalid or removed node");
          return OCCTL_NOT_FOUND;
        }
        aTargets.Append(aId);
      }
    }

    if (aTargets.Size() == 0)
    {
      return OCCTL_OK;
    }

    // Build a compound from the target shapes.
    TopoDS_Compound aCompound;
    BRep_Builder    aBuilder;
    aBuilder.MakeCompound(aCompound);
    for (size_t i = 0; i < aTargets.Size(); ++i)
    {
      const TopoDS_Shape aShape = graph->graph.Shapes().Shape(aTargets[i]);
      if (!aShape.IsNull())
      {
        aBuilder.Add(aCompound, aShape);
      }
    }

    // Invalidate cache before meshing.
    invalidateCache(graph);

    // Mesh the compound shape.
    BRepMesh_IncrementalMesh aMesher(aCompound, aParams);
    if (!aMesher.IsDone())
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_DONE, "BRepMesh_IncrementalMesh failed");
      return OCCTL_NOT_DONE;
    }

    // Push triangulation back onto each graph face that was meshed.
     // BRepMesh_IncrementalMesh stores triangulation on the TShape of each
     // face in the meshed shape.  Since ShapesView reconstructs faces from
     // the same TShapes, the triangulation is visible through
     // BRep_Tool::Triangulation(Shape(faceId), ...).
     const auto& topo = graph->graph.Topo();
     for (auto fid = topo.Faces().StartId(); fid < topo.Faces().EndId(); ++fid)
     {
       const TopoDS_Shape aFaceShape = graph->graph.Shapes().Shape(fid);
       if (aFaceShape.IsNull())
       {
         continue;
       }
       TopLoc_Location                          aLoc;
       const occ::handle<Poly_Triangulation>& aTri =
         BRep_Tool::Triangulation(TopoDS::Face(aFaceShape), aLoc);
       if (aTri.IsNull())
       {
         continue;
       }
       graph->graph.Editor().Faces().SetPersistentTriangulation(fid, aTri);
       graph->graph.Mesh().Editor().Faces().SetCachedTriangulation(fid, aTri);

       // Create polygon-on-triangulation for each coedge of this face.
       for (auto cid = topo.CoEdges().StartId(); cid < topo.CoEdges().EndId(); ++cid)
       {
         const auto& aCoEdgeDef = topo.CoEdges().Definition(cid);
         if (aCoEdgeDef.FaceId != fid)
         {
           continue;
         }
         const BRepGraph_EdgeId aEdgeId = aCoEdgeDef.ChildEdgeId;
         const TopoDS_Edge      aEdge   = TopoDS::Edge(graph->graph.Shapes().Shape(aEdgeId));
         if (aEdge.IsNull())
         {
           continue;
         }
         occ::handle<Poly_PolygonOnTriangulation> aPolyOnTri =
           BRep_Tool::PolygonOnTriangulation(aEdge, aTri, aLoc);
         if (!aPolyOnTri.IsNull())
         {
           graph->graph.Mesh().Editor().CoEdges().AppendCachedPolygonOnTri(cid, aPolyOnTri);
         }
       }
     }

    return OCCTL_OK;
  });
}

} // extern "C"
