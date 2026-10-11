// =============================================================================
// repair.hpp — check and repair the CAD geometry, and put the seam of every
// revolved face at U = 0.
// =============================================================================
#pragma once

#include <TopoDS_Shape.hxx>

// Checks every unique part once (BRepCheck) and repairs the ones that fail
// (ShapeFix); a repaired part replaces every placement of it.
TopoDS_Shape CheckAndRepair(const TopoDS_Shape& shape);

// Rotates every periodic surface of revolution so its parametric origin is at
// the face's seam (U = 0), where OCCT's mesher expects it. The 3D geometry is
// unchanged. Every placement of a part keeps sharing one modified part.
TopoDS_Shape NormalizePeriodicSeams(const TopoDS_Shape& shape, bool verbose = true);
