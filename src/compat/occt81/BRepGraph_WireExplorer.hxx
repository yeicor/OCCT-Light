#ifndef _BRepGraph_WireExplorer_HeaderFile
#define _BRepGraph_WireExplorer_HeaderFile

#include <BRepGraph.hxx>
#include <BRepGraph_DefsIterator.hxx>

#include <cstdint>

class BRepGraph_WireExplorer
{
public:
  BRepGraph_WireExplorer(const BRepGraph& theGraph, const BRepGraph_WireId theWireId)
      : myImpl(theGraph, theWireId),
        myNbEdges(countEdges(theGraph, theWireId))
  {
  }

  bool More() const { return myImpl.More(); }

  void Next() { myImpl.Next(); }

  BRepGraph_CoEdgeId CurrentCoEdgeId() const { return myImpl.CurrentId(); }

  BRepGraph_NodeId Current() const
  {
    const BRepGraph_CoEdgeId aId = myImpl.CurrentId();
    return BRepGraph_NodeId(BRepGraph_NodeId::Kind::CoEdge, aId.Index);
  }

  int NbEdges() const { return myNbEdges; }

private:
  static int countEdges(const BRepGraph& theGraph, const BRepGraph_WireId theWireId)
  {
    int aCount = 0;
    for (BRepGraph_DefsCoEdgeOfWire anIter(theGraph, theWireId); anIter.More(); anIter.Next())
    {
      ++aCount;
    }
    return aCount;
  }

  BRepGraph_DefsCoEdgeOfWire myImpl;
  int                        myNbEdges;
};

#endif
