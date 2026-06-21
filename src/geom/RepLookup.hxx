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

#ifndef OCCTL_GEOM_REPLOOKUP_HXX
#define OCCTL_GEOM_REPLOOKUP_HXX

#include "../compat/occt81/RepsCompat.hxx"
#include "../core/ErrorState.hxx"
#include "../topo/GraphHandle.hxx"
#include "../topo/TopoMath.hxx"

#include <Geom2d_Curve.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>

namespace OcctL::Geom
{

namespace
{

inline BRepGraph_EdgeId FindEdgeByCurve3DRep(const BRepGraph::TopoView& theTopo, uint32_t theIndex)
{
  for (auto aId = theTopo.Edges().StartId(); aId < theTopo.Edges().EndId(); ++aId)
  {
    if (theTopo.Edges().Definition(aId).Curve3DRepId.IsValid()
        && theTopo.Edges().Definition(aId).Curve3DRepId.Index == theIndex)
    {
      return aId;
    }
  }
  return BRepGraph_EdgeId();
}

inline BRepGraph_FaceId FindFaceBySurfaceRep(const BRepGraph::TopoView& theTopo, uint32_t theIndex)
{
  for (auto aId = theTopo.Faces().StartId(); aId < theTopo.Faces().EndId(); ++aId)
  {
    if (theTopo.Faces().Definition(aId).SurfaceRepId.IsValid()
        && theTopo.Faces().Definition(aId).SurfaceRepId.Index == theIndex)
    {
      return aId;
    }
  }
  return BRepGraph_FaceId();
}

inline BRepGraph_CoEdgeId FindCoEdgeByCurve2DRep(const BRepGraph::TopoView& theTopo, uint32_t theIndex)
{
  for (auto aId = theTopo.CoEdges().StartId(); aId < theTopo.CoEdges().EndId(); ++aId)
  {
    if (theTopo.CoEdges().Definition(aId).Curve2DRepId.IsValid()
        && theTopo.CoEdges().Definition(aId).Curve2DRepId.Index == theIndex)
    {
      return aId;
    }
  }
  return BRepGraph_CoEdgeId();
}

} // namespace

inline occ::handle<Geom_Curve> CurveFromRep(const occtl_graph_t* theGraph, occtl_rep_id_t theId)
{
  const BRepGraph_RepId aRawId = OcctL::Topo::UnpackRepId(theId);
  if (theGraph == nullptr || !aRawId.IsValid())
  {
    OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_FOUND, "curve rep id is invalid");
    return occ::handle<Geom_Curve>();
  }
  if (aRawId.RepKind != BRepGraph_RepId::Kind::EdgeCurve3D)
  {
    OcctL::Core::ErrorState::Current().Set(OCCTL_WRONG_KIND, "rep id is not a Curve3D");
    return occ::handle<Geom_Curve>();
  }
  auto& registry = OcctL::Compat::Curve3DRegistryInstance();
  auto* entry = registry.FindByIndex(aRawId.Index);
  if (entry) {
    return entry->Curve;
  }
  BRepGraph_EdgeId aEdgeId = FindEdgeByCurve3DRep(theGraph->graph.Topo(), aRawId.Index);
  if (aEdgeId.IsValid()) {
    return theGraph->graph.Topo().Edges().Curve3D(aEdgeId);
  }
  OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_FOUND, "no curve owns this rep");
  return occ::handle<Geom_Curve>();
}

inline occ::handle<Geom_Surface> SurfaceFromRep(const occtl_graph_t* theGraph, occtl_rep_id_t theId)
{
  const BRepGraph_RepId aRawId = OcctL::Topo::UnpackRepId(theId);
  if (theGraph == nullptr || !aRawId.IsValid())
  {
    OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_FOUND, "surface rep id is invalid");
    return occ::handle<Geom_Surface>();
  }
  if (aRawId.RepKind != BRepGraph_RepId::Kind::FaceSurface)
  {
    OcctL::Core::ErrorState::Current().Set(OCCTL_WRONG_KIND, "rep id is not a Surface");
    return occ::handle<Geom_Surface>();
  }
  BRepGraph_FaceId aFaceId = FindFaceBySurfaceRep(theGraph->graph.Topo(), aRawId.Index);
  if (!aFaceId.IsValid())
  {
    OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_FOUND, "no face owns this surface rep");
    return occ::handle<Geom_Surface>();
  }
  return theGraph->graph.Topo().Faces().Surface(aFaceId);
}

inline occ::handle<Geom2d_Curve> Curve2DFromRep(const occtl_graph_t* theGraph, occtl_rep_id_t theId)
{
  const BRepGraph_RepId aRawId = OcctL::Topo::UnpackRepId(theId);
  if (theGraph == nullptr || !aRawId.IsValid())
  {
    OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_FOUND, "curve2d rep id is invalid");
    return occ::handle<Geom2d_Curve>();
  }
  if (aRawId.RepKind != BRepGraph_RepId::Kind::CoEdgeCurve2D)
  {
    OcctL::Core::ErrorState::Current().Set(OCCTL_WRONG_KIND, "rep id is not a Curve2D");
    return occ::handle<Geom2d_Curve>();
  }
  auto& registry = OcctL::Compat::Curve2DRegistryInstance();
  auto* entry = registry.FindByIndex(aRawId.Index);
  if (entry) {
    return entry->Curve;
  }
  BRepGraph_CoEdgeId aCoEdgeId = FindCoEdgeByCurve2DRep(theGraph->graph.Topo(), aRawId.Index);
  if (aCoEdgeId.IsValid()) {
    return theGraph->graph.Topo().CoEdges().Curve2D(aCoEdgeId);
  }
  OcctL::Core::ErrorState::Current().Set(OCCTL_NOT_FOUND, "no curve2d owns this rep");
  return occ::handle<Geom2d_Curve>();
}

//! Direct lookup overloads for typed rep IDs (no ABI packing).

inline occ::handle<Geom_Curve>
CurveFromRep(const BRepGraph& theGraph, const BRepGraph_EdgeCurve3DRepId& theId)
{
  auto& registry = OcctL::Compat::Curve3DRegistryInstance();
  auto* entry = registry.FindByIndex(theId.Index);
  if (entry) {
    return entry->Curve;
  }
  const auto& topo = theGraph.Topo();
  for (auto eid = topo.Edges().StartId(); eid < topo.Edges().EndId(); ++eid)
  {
    if (topo.Edges().Definition(eid).Curve3DRepId == theId)
      return topo.Edges().Curve3D(eid);
  }
  return occ::handle<Geom_Curve>();
}

inline occ::handle<Geom_Surface>
SurfaceFromRep(const BRepGraph& theGraph, const BRepGraph_FaceSurfaceRepId& theId)
{
  const auto& topo = theGraph.Topo();
  for (auto fid = topo.Faces().StartId(); fid < topo.Faces().EndId(); ++fid)
  {
    if (topo.Faces().Definition(fid).SurfaceRepId == theId)
      return topo.Faces().Surface(fid);
  }
  return occ::handle<Geom_Surface>();
}

inline occ::handle<Geom2d_Curve>
Curve2DFromRep(const BRepGraph& theGraph, const BRepGraph_CoEdgeCurve2DRepId& theId)
{
  auto& registry = OcctL::Compat::Curve2DRegistryInstance();
  auto* entry = registry.FindByIndex(theId.Index);
  if (entry) {
    return entry->Curve;
  }
  const auto& topo = theGraph.Topo();
  for (auto cid = topo.CoEdges().StartId(); cid < topo.CoEdges().EndId(); ++cid)
  {
    if (topo.CoEdges().Definition(cid).Curve2DRepId == theId)
      return topo.CoEdges().Curve2D(cid);
  }
  return occ::handle<Geom2d_Curve>();
}

} // namespace OcctL::Geom

#endif // OCCTL_GEOM_REPLOOKUP_HXX
