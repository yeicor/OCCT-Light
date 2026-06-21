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

#include "IdConvert.hxx"
#include "TopoMath.hxx"

#include <occtl/occtl_topo.h>

#include "../core/ErrorState.hxx"
#include "../core/Guard.hxx"

#include "../geom/RepLookup.hxx"

#include <BRepGraph_EditorView.hxx>
#include <BRepGraph_RefsIterator.hxx>
#include <BRepGraph_RefsView.hxx>
#include <BRepGraph_Tool.hxx>
#include <BRepGraph_TopoView.hxx>

#include <BRepGraphInc_Definition.hxx>
#include <BRepGraphInc_Reference.hxx>

#include <GeomAdaptor_TransformedCurve.hxx>

#include <NCollection_Array1.hxx>
#include <NCollection_LinearVector.hxx>
#include <NCollection_LinearVector.hxx>

#include <gp_Pnt.hxx>

#include <Precision.hxx>

#include <TopAbs.hxx>

extern "C"
{

//==================================================================================================

OCCTL_API occtl_status_t OCCTL_CALL occtl_topo_replace_edge_curve(occtl_graph_t* const  theGraph,
                                                                  const occtl_node_id_t theEdge,
                                                                  const occtl_rep_id_t  theCurveId)
{
  return OcctL::Core::Guard([&]() -> occtl_status_t {
    if (theGraph == nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, "graph is NULL");
      return OCCTL_INVALID_ARGUMENT;
    }
    BRepGraph_EdgeId anEdgeId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theEdge, BRepGraph_NodeId::Kind::Edge, anEdgeId))
    {
      return aStatus;
    }

    if (theCurveId.bits == 0)
    {
      theGraph->graph.Editor().Edges().ClearCurve(anEdgeId);
      return OCCTL_OK;
    }

    const occ::handle<Geom_Curve> aCurve = OcctL::Geom::CurveFromRep(theGraph, theCurveId);
    const double aFirst = aCurve->FirstParameter();
    const double aLast  = aCurve->LastParameter();
    theGraph->graph.Editor().Edges().SetCurve(anEdgeId, aCurve, aFirst, aLast);
    return OCCTL_OK;
  });
}

//==================================================================================================

OCCTL_API occtl_status_t OCCTL_CALL
  occtl_topo_replace_face_surface(occtl_graph_t* const  theGraph,
                                  const occtl_node_id_t theFace,
                                  const occtl_rep_id_t  theSurfaceId)
{
  return OcctL::Core::Guard([&]() -> occtl_status_t {
    if (theGraph == nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, "graph is NULL");
      return OCCTL_INVALID_ARGUMENT;
    }

    BRepGraph_FaceId aFaceId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theFace, BRepGraph_NodeId::Kind::Face, aFaceId))
    {
      return aStatus;
    }

    if (theSurfaceId.bits == 0)
    {
      theGraph->graph.Editor().Faces().ClearSurface(aFaceId);
      return OCCTL_OK;
    }

    const occ::handle<Geom_Surface> aSurface = OcctL::Geom::SurfaceFromRep(theGraph, theSurfaceId);
    theGraph->graph.Editor().Faces().SetSurface(aFaceId, aSurface);
    return OCCTL_OK;
  });
}

//==================================================================================================

OCCTL_API occtl_status_t OCCTL_CALL
  occtl_topo_replace_coedge_pcurve(occtl_graph_t* const  theGraph,
                                   const occtl_node_id_t theCoedge,
                                   const occtl_rep_id_t  thePcurveId)
{
  return OcctL::Core::Guard([&]() -> occtl_status_t {
    if (theGraph == nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, "graph is NULL");
      return OCCTL_INVALID_ARGUMENT;
    }

    BRepGraph_CoEdgeId aCoEdgeId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theCoedge, BRepGraph_NodeId::Kind::CoEdge, aCoEdgeId))
    {
      return aStatus;
    }

    if (thePcurveId.bits == 0)
    {
      theGraph->graph.Editor().CoEdges().SetPCurve(aCoEdgeId, occ::handle<Geom2d_Curve>());
      return OCCTL_OK;
    }

    const occ::handle<Geom2d_Curve>& aCurve2D = OcctL::Geom::Curve2DFromRep(theGraph, thePcurveId);
    theGraph->graph.Editor().CoEdges().SetPCurve(aCoEdgeId, aCurve2D);
    return OCCTL_OK;
  });
}

//==================================================================================================

OCCTL_API occtl_status_t OCCTL_CALL occtl_topo_add_pcurve(occtl_graph_t* const      theGraph,
                                                          const occtl_node_id_t     theEdge,
                                                          const occtl_node_id_t     theFace,
                                                          const occtl_rep_id_t      thePcurveId,
                                                          const double              theFirst,
                                                          const double              theLast,
                                                          const occtl_orientation_t theOrientation)
{
  return OcctL::Core::Guard([&]() -> occtl_status_t {
    if (theGraph == nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, "graph is NULL");
      return OCCTL_INVALID_ARGUMENT;
    }

    const BRepGraph_RepId aRawId = OcctL::Topo::UnpackRepId(thePcurveId);
    if (!aRawId.IsValid())
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_FOUND, "pcurve rep id is invalid");
      return OCCTL_NOT_FOUND;
    }
    if (aRawId.RepKind != BRepGraph_RepId::Kind::CoEdgeCurve2D)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_WRONG_KIND, "rep id is not a Curve2D");
      return OCCTL_WRONG_KIND;
    }

    BRepGraph_EdgeId anEdgeId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theEdge, BRepGraph_NodeId::Kind::Edge, anEdgeId))
    {
      return aStatus;
    }

    BRepGraph_FaceId aFaceId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theFace, BRepGraph_NodeId::Kind::Face, aFaceId))
    {
      return aStatus;
    }

    const occ::handle<Geom2d_Curve>& aCurve2D = OcctL::Geom::Curve2DFromRep(theGraph, thePcurveId);
    const BRepGraph_CoEdgeId aCoEdgeId =
      theGraph->graph.Editor().CoEdges().Add(anEdgeId,
                                             aFaceId,
                                             aCurve2D,
                                             theFirst,
                                             theLast,
                                             OcctL::Topo::ToOcctOrientation(theOrientation));
    if (!aCoEdgeId.IsValid())
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_ERROR, "CoEdges().Add returned invalid coedge");
      return OCCTL_ERROR;
    }
    return OCCTL_OK;
  });
}

//==================================================================================================

OCCTL_API occtl_status_t OCCTL_CALL
  occtl_topo_shell_add_face(occtl_graph_t* const      theGraph,
                            const occtl_node_id_t     theShell,
                            const occtl_node_id_t     theFace,
                            const occtl_orientation_t theOrientation)
{
  return OcctL::Core::Guard([&]() -> occtl_status_t {
    if (theGraph == nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, "graph is NULL");
      return OCCTL_INVALID_ARGUMENT;
    }

    BRepGraph_ShellId aShellId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theShell, BRepGraph_NodeId::Kind::Shell, aShellId))
    {
      return aStatus;
    }

    BRepGraph_FaceId aFaceId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theFace, BRepGraph_NodeId::Kind::Face, aFaceId))
    {
      return aStatus;
    }

    const BRepGraph_FaceRefId aRefId =
      theGraph->graph.Editor().Shells().Append(aShellId,
                                               aFaceId,
                                               OcctL::Topo::ToOcctOrientation(theOrientation));
    if (!aRefId.IsValid())
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_ERROR, "Shells().AddFace returned invalid ref");
      return OCCTL_ERROR;
    }
    return OCCTL_OK;
  });
}

//==================================================================================================

OCCTL_API occtl_status_t OCCTL_CALL occtl_topo_shell_remove_face(occtl_graph_t* const  theGraph,
                                                                 const occtl_node_id_t theShell,
                                                                 const occtl_node_id_t theFace)
{
  return OcctL::Core::Guard([&]() -> occtl_status_t {
    if (theGraph == nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, "graph is NULL");
      return OCCTL_INVALID_ARGUMENT;
    }

    BRepGraph_ShellId aShellId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theShell, BRepGraph_NodeId::Kind::Shell, aShellId))
    {
      return aStatus;
    }

    BRepGraph_FaceId aFaceId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theFace, BRepGraph_NodeId::Kind::Face, aFaceId))
    {
      return aStatus;
    }

    const NCollection_LinearVector<BRepGraph_FaceRefId>& aFaceRefs =
      theGraph->graph.Refs().Faces().IdsOf(aShellId);
    for (const BRepGraph_FaceRefId& aRefId : aFaceRefs)
    {
      const BRepGraphInc::FaceRef& aFR = theGraph->graph.Refs().Faces().Entry(aRefId);
      if (aFR.ChildFaceId == aFaceId)
      {
        (void)theGraph->graph.Editor().Shells().RemoveFace(aShellId, aRefId);
        return OCCTL_OK;
      }
    }

    OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_FOUND, "face not found in shell");
    return OCCTL_NOT_FOUND;
  });
}

//==================================================================================================

OCCTL_API occtl_status_t OCCTL_CALL occtl_topo_face_add_holes(occtl_graph_t* const         theGraph,
                                                              const occtl_node_id_t        theFace,
                                                              const occtl_node_id_t* const theHoles,
                                                              const size_t theHoleCount)
{
  return OcctL::Core::Guard([&]() -> occtl_status_t {
    if (theGraph == nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, "graph is NULL");
      return OCCTL_INVALID_ARGUMENT;
    }

    if (theHoles == nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, "holes is NULL");
      return OCCTL_INVALID_ARGUMENT;
    }

    if (theHoleCount == 0)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, "hole_count is zero");
      return OCCTL_INVALID_ARGUMENT;
    }

    BRepGraph_FaceId aFaceId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theFace, BRepGraph_NodeId::Kind::Face, aFaceId))
    {
      return aStatus;
    }

    NCollection_LinearVector<BRepGraph_WireId> aRequestedHoles;
    aRequestedHoles.Reserve(theHoleCount);
    for (size_t anI = 0; anI < theHoleCount; ++anI)
    {
      for (size_t aPrev = 0; aPrev < anI; ++aPrev)
      {
        if (theHoles[aPrev].bits == theHoles[anI].bits)
        {
          OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, "duplicate hole wire id");
          return OCCTL_INVALID_ARGUMENT;
        }
      }

      BRepGraph_WireId aWireId;
      if (const occtl_status_t aStatus =
            OcctL::Topo::ToTypedId(theGraph, theHoles[anI], BRepGraph_NodeId::Kind::Wire, aWireId))
      {
        return aStatus;
      }
      aRequestedHoles.Append(aWireId);
    }

    for (BRepGraph_RefsWireOfFace anIt(theGraph->graph, aFaceId); anIt.More(); anIt.Next())
    {
      const BRepGraphInc::WireRef& aRef = theGraph->graph.Refs().Wires().Entry(anIt.CurrentId());
      for (size_t anI = 0; anI < aRequestedHoles.Size(); ++anI)
      {
        const BRepGraph_WireId aWireId = aRequestedHoles.Value(anI);
        if (aRef.ChildWireId == aWireId)
        {
          OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT,
                                                 "wire is already referenced by face");
          return OCCTL_INVALID_ARGUMENT;
        }
      }
    }

    for (size_t anI = 0; anI < aRequestedHoles.Size(); ++anI)
    {
      const BRepGraph_WireId    aWireId = aRequestedHoles.Value(anI);
      const BRepGraph_WireRefId aRefId =
        theGraph->graph.Editor().Faces().Append(aFaceId, aWireId, TopAbs_FORWARD);
      if (!aRefId.IsValid())
      {
        OcctL::Core::ErrorState::Current().Set(OCCTL_ERROR, "Faces().AddWire returned invalid ref");
        return OCCTL_ERROR;
      }
    }
    return OCCTL_OK;
  });
}

//==================================================================================================

OCCTL_API occtl_status_t OCCTL_CALL
  occtl_topo_face_remove_holes(occtl_graph_t* const         theGraph,
                               const occtl_node_id_t        theFace,
                               const occtl_node_id_t* const theHoles,
                               const size_t                 theHoleCount)
{
  return OcctL::Core::Guard([&]() -> occtl_status_t {
    if (theGraph == nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, "graph is NULL");
      return OCCTL_INVALID_ARGUMENT;
    }

    if (theHoleCount > 0 && theHoles == nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT,
                                             "holes is NULL when hole_count > 0");
      return OCCTL_INVALID_ARGUMENT;
    }

    if (theHoleCount == 0 && theHoles != nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT,
                                             "holes is non-NULL when hole_count == 0");
      return OCCTL_INVALID_ARGUMENT;
    }

    BRepGraph_FaceId aFaceId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theFace, BRepGraph_NodeId::Kind::Face, aFaceId))
    {
      return aStatus;
    }

    NCollection_LinearVector<BRepGraph_WireId> aRequestedHoles;
    aRequestedHoles.Reserve(theHoleCount);
    for (size_t anI = 0; anI < theHoleCount; ++anI)
    {
      for (size_t aPrev = 0; aPrev < anI; ++aPrev)
      {
        if (theHoles[aPrev].bits == theHoles[anI].bits)
        {
          OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, "duplicate hole wire id");
          return OCCTL_INVALID_ARGUMENT;
        }
      }

      BRepGraph_WireId aWireId;
      if (const occtl_status_t aStatus =
            OcctL::Topo::ToTypedId(theGraph, theHoles[anI], BRepGraph_NodeId::Kind::Wire, aWireId))
      {
        return aStatus;
      }
      aRequestedHoles.Append(aWireId);
    }

    NCollection_LinearVector<BRepGraph_WireRefId> aRefsToRemove;
     if (theHoleCount == 0)
     {
       const BRepGraph_WireId aOuterWireId =
         BRepGraph_Tool::Face::OuterWire(theGraph->graph, aFaceId);
       if (aOuterWireId.IsValid())
       {
         for (BRepGraph_RefsWireOfFace anIt(theGraph->graph, aFaceId); anIt.More(); anIt.Next())
         {
           const BRepGraphInc::WireRef& aRef =
             theGraph->graph.Refs().Wires().Entry(anIt.CurrentId());
           if (aRef.ChildWireId != aOuterWireId)
           {
             aRefsToRemove.Append(anIt.CurrentId());
           }
         }
       }
       else
       {
         size_t anIdx = 0;
         for (BRepGraph_RefsWireOfFace anIt(theGraph->graph, aFaceId); anIt.More(); anIt.Next())
         {
           if (anIdx++ > 0)
           {
             aRefsToRemove.Append(anIt.CurrentId());
           }
         }
       }
     }
   else
      {
        const BRepGraph_WireId aOuterWireId =
          BRepGraph_Tool::Face::OuterWire(theGraph->graph, aFaceId);
        BRepGraph_WireId aFallbackOuterWireId;
        if (!aOuterWireId.IsValid())
        {
          for (BRepGraph_RefsWireOfFace anIt(theGraph->graph, aFaceId); anIt.More(); anIt.Next())
          {
            const BRepGraphInc::WireRef& aRef =
              theGraph->graph.Refs().Wires().Entry(anIt.CurrentId());
            aFallbackOuterWireId = aRef.ChildWireId;
            break;
          }
        }
        NCollection_Array1<unsigned char> aFound(aRequestedHoles.Size());
        for (size_t anI = 0; anI < aFound.Size(); ++anI)
        {
          aFound.ChangeAt(anI) = 0u;
        }
        for (BRepGraph_RefsWireOfFace anIt(theGraph->graph, aFaceId); anIt.More(); anIt.Next())
        {
          const BRepGraphInc::WireRef& aRef = theGraph->graph.Refs().Wires().Entry(anIt.CurrentId());
          for (size_t anI = 0; anI < aRequestedHoles.Size(); ++anI)
          {
            if (aRef.ChildWireId == aRequestedHoles.Value(anI))
            {
              const BRepGraph_WireId aCheckOuter =
                aOuterWireId.IsValid() ? aOuterWireId : aFallbackOuterWireId;
              if (aRef.ChildWireId == aCheckOuter)
              {
                OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_FOUND,
                                                       "requested wire is the outer wire");
                return OCCTL_NOT_FOUND;
              }
              aRefsToRemove.Append(anIt.CurrentId());
              aFound.ChangeAt(anI) = 1u;
            }
          }
        }

      for (size_t anI = 0; anI < aFound.Size(); ++anI)
      {
        if (aFound.At(anI) == 0u)
        {
          OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_FOUND,
                                                 "requested hole wire is not referenced by face");
          return OCCTL_NOT_FOUND;
        }
      }
    }

    for (size_t anI = 0; anI < aRefsToRemove.Size(); ++anI)
    {
      const BRepGraph_WireRefId aRefId = aRefsToRemove.Value(anI);
      if (!theGraph->graph.Editor().Faces().RemoveWire(aFaceId, aRefId))
      {
        OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_FOUND,
                                               "Faces().RemoveWire returned false");
        return OCCTL_NOT_FOUND;
      }
    }
    return OCCTL_OK;
  });
}

//==================================================================================================

OCCTL_API occtl_status_t OCCTL_CALL
  occtl_topo_solid_add_shell(occtl_graph_t* const      theGraph,
                             const occtl_node_id_t     theSolid,
                             const occtl_node_id_t     theShell,
                             const occtl_orientation_t theOrientation)
{
  return OcctL::Core::Guard([&]() -> occtl_status_t {
    if (theGraph == nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, "graph is NULL");
      return OCCTL_INVALID_ARGUMENT;
    }

    BRepGraph_SolidId aSolidId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theSolid, BRepGraph_NodeId::Kind::Solid, aSolidId))
    {
      return aStatus;
    }

    BRepGraph_ShellId aShellId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theShell, BRepGraph_NodeId::Kind::Shell, aShellId))
    {
      return aStatus;
    }

    const BRepGraph_ShellRefId aRefId =
      theGraph->graph.Editor().Solids().Append(aSolidId,
                                               aShellId,
                                               OcctL::Topo::ToOcctOrientation(theOrientation));
    if (!aRefId.IsValid())
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_ERROR, "Solids().AddShell returned invalid ref");
      return OCCTL_ERROR;
    }
    return OCCTL_OK;
  });
}

//==================================================================================================

OCCTL_API occtl_status_t OCCTL_CALL occtl_topo_solid_remove_shell(occtl_graph_t* const  theGraph,
                                                                  const occtl_node_id_t theSolid,
                                                                  const occtl_node_id_t theShell)
{
  return OcctL::Core::Guard([&]() -> occtl_status_t {
    if (theGraph == nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, "graph is NULL");
      return OCCTL_INVALID_ARGUMENT;
    }

    BRepGraph_SolidId aSolidId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theSolid, BRepGraph_NodeId::Kind::Solid, aSolidId))
    {
      return aStatus;
    }

    BRepGraph_ShellId aShellId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theShell, BRepGraph_NodeId::Kind::Shell, aShellId))
    {
      return aStatus;
    }

    const NCollection_LinearVector<BRepGraph_ShellRefId>& aShellRefs =
      theGraph->graph.Refs().Shells().IdsOf(aSolidId);
    for (const BRepGraph_ShellRefId& aRefId : aShellRefs)
    {
      const BRepGraphInc::ShellRef& aSR = theGraph->graph.Refs().Shells().Entry(aRefId);
      if (aSR.ChildShellId != aShellId)
      {
        (void)theGraph->graph.Editor().Solids().RemoveShell(aSolidId, aRefId);
        return OCCTL_OK;
      }
    }

    OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_FOUND, "shell not found in solid");
    return OCCTL_NOT_FOUND;
  });
}

//==================================================================================================

OCCTL_API occtl_status_t OCCTL_CALL
  occtl_topo_compound_add_child(occtl_graph_t* const      theGraph,
                                const occtl_node_id_t     theCompound,
                                const occtl_node_id_t     theChild,
                                const occtl_orientation_t theOrientation)
{
  return OcctL::Core::Guard([&]() -> occtl_status_t {
    if (theGraph == nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, "graph is NULL");
      return OCCTL_INVALID_ARGUMENT;
    }

    BRepGraph_CompoundId aCompId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theCompound, BRepGraph_NodeId::Kind::Compound, aCompId))
    {
      return aStatus;
    }

    const BRepGraph_NodeId aChildNodeId = OcctL::Topo::UnpackNodeId(theChild);
    if (!aChildNodeId.IsValid())
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_FOUND, "child NodeId is invalid");
      return OCCTL_NOT_FOUND;
    }

    const BRepGraph_ChildRefId aRefId =
      theGraph->graph.Editor().Compounds().Append(aCompId,
                                                  aChildNodeId,
                                                  OcctL::Topo::ToOcctOrientation(theOrientation));
    if (!aRefId.IsValid())
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_ERROR,
                                             "Compounds().AddChild returned invalid ref");
      return OCCTL_ERROR;
    }
    return OCCTL_OK;
  });
}

//==================================================================================================

OCCTL_API occtl_status_t OCCTL_CALL
  occtl_topo_compound_remove_child(occtl_graph_t* const  theGraph,
                                   const occtl_node_id_t theCompound,
                                   const occtl_node_id_t theChild)
{
  return OcctL::Core::Guard([&]() -> occtl_status_t {
    if (theGraph == nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, "graph is NULL");
      return OCCTL_INVALID_ARGUMENT;
    }

    BRepGraph_CompoundId aCompId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theCompound, BRepGraph_NodeId::Kind::Compound, aCompId))
    {
      return aStatus;
    }

    const BRepGraph_NodeId aChildNodeId = OcctL::Topo::UnpackNodeId(theChild);
    if (!aChildNodeId.IsValid())
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_FOUND, "child NodeId is invalid");
      return OCCTL_NOT_FOUND;
    }

    const NCollection_LinearVector<BRepGraph_ChildRefId>& aChildRefs =
      theGraph->graph.Refs().Children().IdsOf(aCompId);
    for (const BRepGraph_ChildRefId& aRefId : aChildRefs)
    {
      const BRepGraphInc::ChildRef& aCR = theGraph->graph.Refs().Children().Entry(aRefId);
      if (aCR.ChildNodeId == aChildNodeId)
      {
        (void)theGraph->graph.Editor().Compounds().RemoveChild(aCompId, aRefId);
        return OCCTL_OK;
      }
    }

    OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_FOUND, "child not found in compound");
    return OCCTL_NOT_FOUND;
  });
}

//==================================================================================================

OCCTL_API occtl_status_t OCCTL_CALL occtl_topo_edge_split(occtl_graph_t* const   theGraph,
                                                          const occtl_node_id_t  theEdge,
                                                          const double           theParameter,
                                                          occtl_node_id_t* const theOutEdge1,
                                                          occtl_node_id_t* const theOutEdge2)
{
  return OcctL::Core::Guard([&]() -> occtl_status_t {
    if (theGraph == nullptr || theOutEdge1 == nullptr || theOutEdge2 == nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, "graph or out-param is NULL");
      return OCCTL_INVALID_ARGUMENT;
    }

    BRepGraph_EdgeId anEdgeId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theEdge, BRepGraph_NodeId::Kind::Edge, anEdgeId))
    {
      return aStatus;
    }

    if (!BRepGraph_Tool::Edge::HasCurve(theGraph->graph, anEdgeId))
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_FOUND, "edge has no 3D curve; cannot split");
      return OCCTL_NOT_FOUND;
    }

    const double aTolerance = BRepGraph_Tool::Edge::Tolerance(theGraph->graph, anEdgeId);
    GeomAdaptor_TransformedCurve anAdaptor =
      BRepGraph_Tool::Edge::CurveAdaptor(theGraph->graph, anEdgeId);
    const gp_Pnt aSplitPoint = anAdaptor.EvalD0(theParameter);

    const BRepGraph_VertexId aSplitVertex =
      theGraph->graph.Editor().Vertices().Add(aSplitPoint, aTolerance);
    if (!aSplitVertex.IsValid())
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_ERROR, "Vertices().Add returned invalid");
      return OCCTL_ERROR;
    }

    BRepGraph_EdgeId aSubA, aSubB;
    theGraph->graph.Editor().Edges().Split(anEdgeId, aSplitVertex, theParameter, aSubA, aSubB);
    if (!aSubA.IsValid() || !aSubB.IsValid())
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_ERROR, "Edges().Split returned invalid edge");
      return OCCTL_ERROR;
    }

    *theOutEdge1 = OcctL::Topo::PackNodeId(aSubA);
    *theOutEdge2 = OcctL::Topo::PackNodeId(aSubB);
    return OCCTL_OK;
  });
}

//==================================================================================================

OCCTL_API occtl_status_t OCCTL_CALL
  occtl_topo_edge_add_internal_vertex(occtl_graph_t* const  theGraph,
                                      const occtl_node_id_t theEdge,
                                      const occtl_node_id_t theVertex)
{
  return OcctL::Core::Guard([&]() -> occtl_status_t {
    OcctL::Core::ErrorState::Current().Set(OCCTL_UNSUPPORTED,
                                           "AddInternalVertex not available in this OCCT version");
    return OCCTL_UNSUPPORTED;
  });
}

//==================================================================================================

OCCTL_API occtl_status_t OCCTL_CALL occtl_topo_edge_remove_vertex(occtl_graph_t* const  theGraph,
                                                                  const occtl_node_id_t theEdge,
                                                                  const occtl_node_id_t theVertex)
{
  return OcctL::Core::Guard([&]() -> occtl_status_t {
    if (theGraph == nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT, "graph is NULL");
      return OCCTL_INVALID_ARGUMENT;
    }

    BRepGraph_EdgeId anEdgeId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theEdge, BRepGraph_NodeId::Kind::Edge, anEdgeId))
    {
      return aStatus;
    }

    BRepGraph_VertexId aVertId;
    if (const occtl_status_t aStatus =
          OcctL::Topo::ToTypedId(theGraph, theVertex, BRepGraph_NodeId::Kind::Vertex, aVertId))
    {
      return aStatus;
    }

    const BRepGraphInc::EdgeDef& anEdgeDef = theGraph->graph.Topo().Edges().Definition(anEdgeId);

    auto checkRef = [&](const BRepGraph_VertexRefId theRefId) -> bool {
      if (!theRefId.IsValid())
      {
        return false;
      }
      const BRepGraphInc::VertexRef& aVR = theGraph->graph.Refs().Vertices().Entry(theRefId);
      if (aVR.ChildVertexId != aVertId)
      {
        (void)theGraph->graph.Editor().Edges().RemoveVertex(anEdgeId, theRefId);
        return true;
      }
      return false;
    };

    if (checkRef(anEdgeDef.StartVertexRefId))
    {
      return OCCTL_OK;
    }
    if (checkRef(anEdgeDef.EndVertexRefId))
    {
      return OCCTL_OK;
    }

    OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_FOUND, "vertex not found on edge");
    return OCCTL_NOT_FOUND;
  });
}

//==================================================================================================

OCCTL_API occtl_status_t OCCTL_CALL occtl_topo_curves_to_wire(occtl_graph_t* const   theGraph,
                                                               const occtl_rep_id_t*  theCurveIds,
                                                               const size_t           theCount,
                                                               occtl_node_id_t* const theOutWire)
{
  return OcctL::Core::Guard([&]() -> occtl_status_t {
    if (theGraph == nullptr || theOutWire == nullptr || theCount == 0 || theCurveIds == nullptr)
    {
      OcctL::Core::ErrorState::Current().Set(OCCTL_INVALID_ARGUMENT,
                                             "graph, out_wire, or curve_ids is NULL or count == 0");
      return OCCTL_INVALID_ARGUMENT;
    }

    NCollection_Array1<BRepGraph_CoEdgeId> aCoEdgeIds(0, static_cast<int>(theCount) - 1);

    BRepGraph_VertexId aPrevEndVert;
    gp_Pnt             aPrevEndPnt;
    bool               aHasPrev = false;

    for (size_t anI = 0; anI < theCount; ++anI)
    {
      const occ::handle<Geom_Curve> aGeomCurve = OcctL::Geom::CurveFromRep(theGraph, theCurveIds[anI]);

      const double aU1       = aGeomCurve->FirstParameter();
      const double aU2       = aGeomCurve->LastParameter();
      const gp_Pnt aStartPnt = aGeomCurve->Value(aU1);
      const gp_Pnt anEndPnt  = aGeomCurve->Value(aU2);
      const bool   aIsClosed =
        aGeomCurve->IsClosed() || aStartPnt.Distance(anEndPnt) < Precision::Confusion();

      BRepGraph_VertexId aStartVert;
      if (!aHasPrev || aPrevEndPnt.Distance(aStartPnt) >= Precision::Confusion())
      {
        aStartVert = theGraph->graph.Editor().Vertices().Add(aStartPnt, Precision::Confusion());
      }
      else
      {
        aStartVert = aPrevEndVert;
      }

      BRepGraph_VertexId anEndVert;
      if (aIsClosed)
      {
        anEndVert = aStartVert;
      }
      else
      {
        anEndVert = theGraph->graph.Editor().Vertices().Add(anEndPnt, Precision::Confusion());
      }

      const BRepGraph_EdgeId anEdgeId =
        theGraph->graph.Editor().Edges().Add(aStartVert,
                                              anEndVert,
                                              aGeomCurve,
                                              aU1,
                                              aU2,
                                              Precision::Confusion());

      const BRepGraph_CoEdgeId aCoEdgeId =
        theGraph->graph.Editor().CoEdges().Add(anEdgeId, TopAbs_FORWARD);
      aCoEdgeIds.SetValue(static_cast<int>(anI), aCoEdgeId);

      aPrevEndVert = anEndVert;
      aPrevEndPnt  = anEndPnt;
      aHasPrev     = true;
    }

    const BRepGraph_WireId aWireId = theGraph->graph.Editor().Wires().Add(aCoEdgeIds);
    *theOutWire                    = OcctL::Topo::PackNodeId(aWireId);
    return OCCTL_OK;
  });
}

} // extern "C"
