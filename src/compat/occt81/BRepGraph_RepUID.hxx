#ifndef _BRepGraph_RepUID_HeaderFile
#define _BRepGraph_RepUID_HeaderFile

#include <BRepGraphInc_RepId.hxx>

#include <cstddef>
#include <cstdint>

struct BRepGraph_RepUID
{
  BRepGraph_RepId::Kind myKind;
  size_t                myCounter;
  uint32_t              myGeneration;

  BRepGraph_RepUID()
      : myKind(BRepGraph_RepId::Kind::EdgeCurve3D),
        myCounter(0),
        myGeneration(0)
  {
  }

  BRepGraph_RepUID(const BRepGraph_RepId::Kind theKind,
                   const size_t                theCounter,
                   const uint32_t              theGeneration)
      : myKind(theKind),
        myCounter(theCounter),
        myGeneration(theGeneration)
  {
  }

  static BRepGraph_RepUID Invalid() { return BRepGraph_RepUID(); }

  bool IsValid() const { return myCounter != 0; }

  BRepGraph_RepId::Kind Kind() const { return myKind; }

  size_t Counter() const { return myCounter; }

  bool operator==(const BRepGraph_RepUID& theOther) const
  {
    return myKind == theOther.myKind && myCounter == theOther.myCounter
           && myGeneration == theOther.myGeneration;
  }

  bool operator!=(const BRepGraph_RepUID& theOther) const { return !(*this == theOther); }
};

#endif
