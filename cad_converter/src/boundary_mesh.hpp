// =============================================================================
// boundary_mesh.hpp — last-resort triangulation for faces BRepMesh can't mesh.
// See boundary_mesh.cpp.
// =============================================================================
#pragma once

#include <TopoDS_Shape.hxx>

// For every face of 'model' that still has no triangulation, builds one
// from the face's boundary alone: the points the neighbouring faces' meshes
// already placed on the shared edges (so the result stays flush with them),
// triangulated by ear clipping. Returns the number of faces triangulated.
int TriangulateUnmeshedFacesFromBoundary(const TopoDS_Shape& model, double deflection);
