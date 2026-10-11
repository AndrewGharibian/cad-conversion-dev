// =============================================================================
// folded_faces.hpp — repair faces that STEP import put on the wrong side of
// a surface of revolution ("folded lunes").
//
// See folded_faces.cpp for the full explanation. In short: a face bounded by
// two meridians of a surface of revolution and nothing but degenerate
// (point) edges otherwise — a torus elbow whose tube passes through the
// axis, a sphere or cone lune — covers either the arc between the meridians
// or its complement, and the 3D boundary can't tell which. When the import
// picks the wrong one, the face folds back onto its neighbour. This finds
// those faces by the fold and rebuilds them on the other arc.
// =============================================================================
#pragma once

#include <TopoDS_Shape.hxx>

// Returns 'shape' with every folded lune rebuilt on the complementary arc
// (every placement of a part at once), and the number of faces fixed.
TopoDS_Shape FixFoldedLunes(const TopoDS_Shape& shape, int& fixedFaces);
