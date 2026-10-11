// =============================================================================
// revolved_mesh.hpp — structured, seam-free triangulation of full-revolution
// cylinders, cones and spheres. See revolved_mesh.cpp.
// =============================================================================
#pragma once

#include <TopoDS_Shape.hxx>

// Replaces OCCT's triangulation of every full-revolution (360 degree)
// cylindrical, conical and spherical face of an already-meshed shape with a
// structured one:
//   cylinder — one ring of rectangles from boundary circle to boundary circle
//   cone     — a fan from the base circle to the apex (or rectangles, for a
//              truncated cone)
//   sphere   — a latitude/longitude grid whose columns are evenly spaced all
//              the way around, so there is no seam strip
// The boundary circles keep exactly the points OCCT put on them, so the
// faces still join their neighbours without cracks. Faces that don't fit
// the pattern (partial revolves, holes, non-circular boundaries) keep OCCT's
// triangulation. Returns the number of faces replaced.
int RetessellateRevolvedFaces(const TopoDS_Shape& meshedShape, double maxDeviation, double maxAngleRad);
