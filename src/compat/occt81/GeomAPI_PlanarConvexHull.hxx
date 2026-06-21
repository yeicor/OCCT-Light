#ifndef _GeomAPI_PlanarConvexHull_HeaderFile
#define _GeomAPI_PlanarConvexHull_HeaderFile

// Stub: GeomAPI_PlanarConvexHull is not available in OCCT 8.0.0-p1.
// This header provides a minimal stub so that code including it compiles.
// Any use of this class at runtime will fail with OCCTL_NOT_IMPLEMENTED.

#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <memory>
#include <vector>

class GeomAPI_PlanarConvexHull
{
public:
  GeomAPI_PlanarConvexHull() {}

  GeomAPI_PlanarConvexHull(const std::vector<gp_Pnt>&,
                           const gp_Ax2&,
                           const double)
  {
  }

  bool IsDone() const { return false; }

  uint32_t NbPoints() const { return 0; }

  gp_Pnt Point(const uint32_t) const { return gp_Pnt(); }

  std::vector<gp_Pnt> Points() const { return {}; }
};

#endif
