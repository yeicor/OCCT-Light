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

#ifndef OCCTL_COMPAT_REPS_COMPAT_HXX
#define OCCTL_COMPAT_REPS_COMPAT_HXX

#include <algorithm>
#include <vector>

#include <BRepGraph_EditorView.hxx>
#include <BRepGraph_TopoView.hxx>
#include <BRepGraphInc_Representation.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <gp_Pnt.hxx>
#include <Poly_Polygon2D.hxx>
#include <Poly_Polygon3D.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <Poly_Triangulation.hxx>
#include <TopAbs_Orientation.hxx>


//! Compatibility helpers for BRepGraph 8.0.0-p1, which exposes
//! mutation only through Editor() and has no direct Reps() accessor.
//! These mirror the Reps() API by using Editor().Edges().Add()
//! and the corresponding operations for each representation kind.

namespace OcctL::Compat
{

namespace
{

//! Helper: create a single vertex in the graph at the given point.
inline BRepGraph_VertexId MakeVertex(BRepGraph& theGraph, const gp_Pnt& theP)
{
  return theGraph.Editor().Vertices().Add(theP, 0.0);
}

} // namespace

//! Storage for curve3D representations without creating topology edges.
struct Curve3DRegistry
{
  struct Entry
  {
    uint32_t Index;
    occ::handle<Geom_Curve> Curve;
    double First, Last;
  };
  std::vector<Entry> curves;

  ~Curve3DRegistry()
  {
    for (auto& c : curves) {
      c.Curve.Nullify();
    }
    curves.clear();
  }

  static Curve3DRegistry& Instance()
  {
    static Curve3DRegistry s_instance;
    return s_instance;
  }

  Entry* FindByIndex(uint32_t theIndex)
  {
    for (auto& c : curves) {
      if (c.Index == theIndex) return &c;
    }
    return nullptr;
  }

  void Remove(uint32_t theIndex)
  {
    curves.erase(
      std::remove_if(curves.begin(), curves.end(),
                     [theIndex](const Entry& e){ return e.Index == theIndex; }),
      curves.end());
  }
};

inline Curve3DRegistry& Curve3DRegistryInstance()
{
  return Curve3DRegistry::Instance();
}

//! Storage for curve2D (PCurve) representations without creating topology edges.
struct Curve2DRegistry
{
  struct Entry
  {
    uint32_t Index;
    occ::handle<Geom2d_Curve> Curve;
    double First, Last;
  };
  std::vector<Entry> curves;

  ~Curve2DRegistry()
  {
    for (auto& c : curves) {
      c.Curve.Nullify();
    }
    curves.clear();
  }

  static Curve2DRegistry& Instance()
  {
    static Curve2DRegistry s_instance;
    return s_instance;
  }

  Entry* FindByIndex(uint32_t theIndex)
  {
    for (auto& c : curves) {
      if (c.Index == theIndex) return &c;
    }
    return nullptr;
  }

  void Remove(uint32_t theIndex)
  {
    curves.erase(
      std::remove_if(curves.begin(), curves.end(),
                     [theIndex](const Entry& e){ return e.Index == theIndex; }),
      curves.end());
  }
};

inline Curve2DRegistry& Curve2DRegistryInstance()
{
  return Curve2DRegistry::Instance();
}

namespace
{

//! Helper: create a Curve3D rep without topology edges; stores curve in a registry.
inline BRepGraph_EdgeCurve3DRepId MakeCurve3DRep(BRepGraph& theGraph,
                                                   const occ::handle<Geom_Curve>& theCurve,
                                                   const double                   theFirst,
                                                   const double                   theLast)
{
  auto& registry = Curve3DRegistryInstance();
  uint32_t idx = static_cast<uint32_t>(registry.curves.size());
  registry.curves.push_back({idx, theCurve, theFirst, theLast});
  return BRepGraph_EdgeCurve3DRepId(idx);
}

//! Helper: build a degenerate quad face structure (4 vertices, 4 edges, 4 coedges, 1 wire, 1 face).
//! Returns the face id. The edges are: 0->1, 1->2, 2->3, 3->0.
inline BRepGraph_FaceId MakeFace(BRepGraph& theGraph,
                                  const gp_Pnt& p1,
                                  const gp_Pnt& p2,
                                  const gp_Pnt& p3,
                                  const gp_Pnt& p4,
                                  const occ::handle<Geom_Surface>& theSurface)
{
  auto& editor = theGraph.Editor();
  BRepGraph_VertexId v1 = MakeVertex(theGraph, p1);
  BRepGraph_VertexId v2 = MakeVertex(theGraph, p2);
  BRepGraph_VertexId v3 = MakeVertex(theGraph, p3);
  BRepGraph_VertexId v4 = MakeVertex(theGraph, p4);

  // Create four edges.
  BRepGraph_EdgeId e1 = editor.Edges().Add(v1, v2, nullptr, 0.0, 1.0, 0.0);
  BRepGraph_EdgeId e2 = editor.Edges().Add(v2, v3, nullptr, 0.0, 1.0, 0.0);
  BRepGraph_EdgeId e3 = editor.Edges().Add(v3, v4, nullptr, 0.0, 1.0, 0.0);
  BRepGraph_EdgeId e4 = editor.Edges().Add(v4, v1, nullptr, 0.0, 1.0, 0.0);

  // Create coedges on the face.
  BRepGraph_CoEdgeId c1 = editor.CoEdges().Add(e1, TopAbs_FORWARD);
  BRepGraph_CoEdgeId c2 = editor.CoEdges().Add(e2, TopAbs_FORWARD);
  BRepGraph_CoEdgeId c3 = editor.CoEdges().Add(e3, TopAbs_FORWARD);
  BRepGraph_CoEdgeId c4 = editor.CoEdges().Add(e4, TopAbs_FORWARD);

  // Create wire from coedges.
  NCollection_Array1<BRepGraph_CoEdgeId> coEdges(0, 3);
  coEdges(0) = c1;
  coEdges(1) = c2;
  coEdges(2) = c3;
  coEdges(3) = c4;

  BRepGraph_WireId w = editor.Wires().Add(coEdges);

  NCollection_Array1<BRepGraph_WireId> innerWires(0, 0);

  return editor.Faces().Add(theSurface, w, innerWires, 0.0);
}

} // namespace

//! Create a new Curve3D rep and return its identifier.
inline BRepGraph_EdgeCurve3DRepId CreateCurve3DRep(BRepGraph& theGraph,
                                                     const occ::handle<Geom_Curve>& theCurve)
{
  return MakeCurve3DRep(theGraph, theCurve, theCurve->FirstParameter(), theCurve->LastParameter());
}

//! Create a new Curve2D (PCurve) rep and return its identifier.
inline BRepGraph_CoEdgeCurve2DRepId CreateCurve2DRep(BRepGraph& theGraph,
                                                        const occ::handle<Geom2d_Curve>& theCurve)
{
  auto& registry = Curve2DRegistryInstance();
  uint32_t idx = static_cast<uint32_t>(registry.curves.size());
  registry.curves.push_back({idx, theCurve, theCurve->FirstParameter(), theCurve->LastParameter()});
  return BRepGraph_CoEdgeCurve2DRepId(idx);
}

//! Create a new Surface rep and return its identifier.
inline BRepGraph_FaceSurfaceRepId CreateSurfaceRep(BRepGraph& theGraph,
                                                     const occ::handle<Geom_Surface>& theSurface)
{
  BRepGraph_FaceId f =
      MakeFace(theGraph, gp_Pnt(0, 0, 0), gp_Pnt(1, 0, 0), gp_Pnt(1, 1, 0), gp_Pnt(0, 1, 0),
               theSurface);
  theGraph.Editor().Faces().SetSurface(f, theSurface);
  return theGraph.Topo().Faces().Definition(f).SurfaceRepId;
}

//! Create a new Triangulation rep and return its identifier.
inline BRepGraph_FaceTriangulationRepId CreateTriangulationRep(
    BRepGraph& theGraph, const occ::handle<Poly_Triangulation>& theTriangulation)
{
  auto& editor = theGraph.Editor();

  // Build a minimal face structure.
  BRepGraph_VertexId v1 = MakeVertex(theGraph, gp_Pnt(0, 0, 0));
  BRepGraph_VertexId v2 = MakeVertex(theGraph, gp_Pnt(1, 0, 0));
  BRepGraph_VertexId v3 = MakeVertex(theGraph, gp_Pnt(1, 1, 0));
  BRepGraph_VertexId v4 = MakeVertex(theGraph, gp_Pnt(0, 1, 0));

  BRepGraph_EdgeId e1 = editor.Edges().Add(v1, v2, nullptr, 0.0, 1.0, 0.0);
  BRepGraph_EdgeId e2 = editor.Edges().Add(v2, v3, nullptr, 0.0, 1.0, 0.0);
  BRepGraph_EdgeId e3 = editor.Edges().Add(v3, v4, nullptr, 0.0, 1.0, 0.0);
  BRepGraph_EdgeId e4 = editor.Edges().Add(v4, v1, nullptr, 0.0, 1.0, 0.0);

  BRepGraph_CoEdgeId c1 = editor.CoEdges().Add(e1, TopAbs_FORWARD);
  BRepGraph_CoEdgeId c2 = editor.CoEdges().Add(e2, TopAbs_FORWARD);
  BRepGraph_CoEdgeId c3 = editor.CoEdges().Add(e3, TopAbs_FORWARD);
  BRepGraph_CoEdgeId c4 = editor.CoEdges().Add(e4, TopAbs_FORWARD);

  NCollection_Array1<BRepGraph_CoEdgeId> coEdges(0, 3);
  coEdges(0) = c1;
  coEdges(1) = c2;
  coEdges(2) = c3;
  coEdges(3) = c4;

  BRepGraph_WireId w = editor.Wires().Add(coEdges);
  NCollection_Array1<BRepGraph_WireId> innerWires(0, 0);

  BRepGraph_FaceId f =
      editor.Faces().Add(occ::handle<Geom_Surface>(), w, innerWires, 0.0);

  editor.Faces().SetPersistentTriangulation(f, theTriangulation);

  return theGraph.Topo().Faces().Definition(f).TriangulationRepId;
}

//! Create a new 3D Polygon rep and return its identifier.
inline BRepGraph_EdgePolygon3DRepId CreatePolygon3DRep(
    BRepGraph& theGraph, const occ::handle<Poly_Polygon3D>& thePolygon)
{
  auto& editor = theGraph.Editor();

  BRepGraph_VertexId v = MakeVertex(theGraph, gp_Pnt(0, 0, 0));
  BRepGraph_EdgeId e = editor.Edges().Add(v, v, nullptr, 0.0, 1.0, 0.0);
  editor.Edges().SetPersistentPolygon3D(e, thePolygon);

  return theGraph.Topo().Edges().Definition(e).Polygon3DRepId;
}

//! Create a new 2D Polygon rep and return its identifier.
inline BRepGraph_CoEdgePolygon2DRepId CreatePolygon2DRep(
    BRepGraph& theGraph, const occ::handle<Poly_Polygon2D>& thePolygon)
{
  auto& editor = theGraph.Editor();

  BRepGraph_VertexId v = MakeVertex(theGraph, gp_Pnt(0, 0, 0));
  BRepGraph_EdgeId e = editor.Edges().Add(v, v, nullptr, 0.0, 1.0, 0.0);
  BRepGraph_CoEdgeId c = editor.CoEdges().Add(e, TopAbs_FORWARD);
  editor.CoEdges().SetPersistentPolygon2D(c, thePolygon);

  return theGraph.Topo().CoEdges().Definition(c).Polygon2DRepId;
}

//! Create a new Polygon-on-Triangulation rep and return its identifier.
inline BRepGraph_CoEdgePolygonOnTriRepId CreatePolygonOnTriRep(
    BRepGraph& theGraph, const occ::handle<Poly_PolygonOnTriangulation>& thePolygon)
{
  auto& editor = theGraph.Editor();

  // Build a minimal face structure for the polygon-on-triangulation.
  BRepGraph_VertexId v1 = MakeVertex(theGraph, gp_Pnt(0, 0, 0));
  BRepGraph_VertexId v2 = MakeVertex(theGraph, gp_Pnt(1, 0, 0));
  BRepGraph_VertexId v3 = MakeVertex(theGraph, gp_Pnt(1, 1, 0));
  BRepGraph_VertexId v4 = MakeVertex(theGraph, gp_Pnt(0, 1, 0));

  BRepGraph_EdgeId e1 = editor.Edges().Add(v1, v2, nullptr, 0.0, 1.0, 0.0);
  BRepGraph_EdgeId e2 = editor.Edges().Add(v2, v3, nullptr, 0.0, 1.0, 0.0);
  BRepGraph_EdgeId e3 = editor.Edges().Add(v3, v4, nullptr, 0.0, 1.0, 0.0);
  BRepGraph_EdgeId e4 = editor.Edges().Add(v4, v1, nullptr, 0.0, 1.0, 0.0);

  BRepGraph_CoEdgeId c1 = editor.CoEdges().Add(e1, TopAbs_FORWARD);
  BRepGraph_CoEdgeId c2 = editor.CoEdges().Add(e2, TopAbs_FORWARD);
  BRepGraph_CoEdgeId c3 = editor.CoEdges().Add(e3, TopAbs_FORWARD);
  BRepGraph_CoEdgeId c4 = editor.CoEdges().Add(e4, TopAbs_FORWARD);

  NCollection_Array1<BRepGraph_CoEdgeId> coEdges(0, 3);
  coEdges(0) = c1;
  coEdges(1) = c2;
  coEdges(2) = c3;
  coEdges(3) = c4;

  BRepGraph_WireId w = editor.Wires().Add(coEdges);
  NCollection_Array1<BRepGraph_WireId> innerWires(0, 0);

  BRepGraph_FaceId f =
      editor.Faces().Add(occ::handle<Geom_Surface>(), w, innerWires, 0.0);

  // Create a coedge with the polygon-on-triangulation.
  BRepGraph_CoEdgeId c = editor.CoEdges().Add(e1, f, nullptr, 0.0, 1.0);
  editor.CoEdges().SetPersistentPolygonOnTri(c, thePolygon);

  return theGraph.Topo().CoEdges().Definition(c).PolygonOnTriRepId;
}

} // namespace OcctL::Compat

#endif // OCCTL_COMPAT_REPS_COMPAT_HXX
