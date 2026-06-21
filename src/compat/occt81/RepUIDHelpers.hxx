#ifndef OCCTL_COMPAT_REPUID_HELPERS_HXX
#define OCCTL_COMPAT_REPUID_HELPERS_HXX

#include <BRepGraph.hxx>
#include <BRepGraph_RepUID.hxx>

namespace OcctL {
namespace Compat {

inline BRepGraph_RepUID RepIdToRepUID(const BRepGraph& theGraph, const BRepGraph_RepId theRepId)
{
  if (!theRepId.IsValid())
  {
    return BRepGraph_RepUID::Invalid();
  }
  size_t aCounter = static_cast<size_t>(theRepId.Index) + 1;
  return BRepGraph_RepUID(theRepId.RepKind, aCounter, 0);
}

inline bool UidsHas(const BRepGraph& theGraph, const BRepGraph_RepUID& theUid)
{
  if (!theUid.IsValid())
  {
    return false;
  }
  const size_t anIndex = theUid.Counter() - 1;
  BRepGraph_RepId aRepId(theUid.Kind(), static_cast<uint32_t>(anIndex));
  if (!aRepId.IsValid())
  {
    return false;
  }
  const BRepGraph_NodeId::Kind aNodeKind = [](const BRepGraph_RepId::Kind theKind) {
    switch (theKind)
    {
      case BRepGraph_RepId::Kind::FaceSurface:
      case BRepGraph_RepId::Kind::FaceTriangulation:
        return BRepGraph_NodeId::Kind::Face;
      case BRepGraph_RepId::Kind::EdgeCurve3D:
      case BRepGraph_RepId::Kind::EdgePolygon3D:
        return BRepGraph_NodeId::Kind::Edge;
      case BRepGraph_RepId::Kind::CoEdgeCurve2D:
      case BRepGraph_RepId::Kind::CoEdgePolygon2D:
      case BRepGraph_RepId::Kind::CoEdgePolygonOnTri:
        return BRepGraph_NodeId::Kind::CoEdge;
    }
    return BRepGraph_NodeId::Kind::Vertex;
  }(theUid.Kind());

  if (!theGraph.Topo().Gen().IsActive(BRepGraph_NodeId(aNodeKind, static_cast<uint32_t>(anIndex))))
  {
    return false;
  }

  return !aRepId.IsRemoved(theGraph);
}

inline BRepGraph_RepId UidToRepId(const BRepGraph& theGraph, const BRepGraph_RepUID& theUid)
{
  if (!theUid.IsValid())
  {
    return BRepGraph_RepId();
  }
  const size_t anIndex = theUid.Counter() - 1;
  return BRepGraph_RepId(theUid.Kind(), static_cast<uint32_t>(anIndex));
}

} // namespace Compat
} // namespace OcctL

#endif
