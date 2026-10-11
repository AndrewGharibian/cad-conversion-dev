// =============================================================================
// meshing.hpp — triangulate the B-rep with OCCT, with fallbacks for faces
// the mesher can't handle.
// =============================================================================
#pragma once

#include "body.hpp"

#include <TopoDS_Shape.hxx>

#include <vector>

constexpr double kPi = 3.14159265358979323846; // <cmath>'s M_PI isn't standard on MSVC

// Largest angle between neighbouring triangles along a curve, for OCCT's
// mesher and the structured revolved faces alike.
constexpr double kMeshAngleDeg = 20.0;

// The default accuracy limit: 1/1000 of the shape's largest dimension.
double DefaultMaxDeviation(const TopoDS_Shape& shape);

// Meshes every face within 'deflection' of the exact surface, retrying the
// faces the mesher gives up on. Returns the shape that carries the
// triangulation, which may be a rebuilt copy: use it from then on.
TopoDS_Shape MeshWithFallback(const TopoDS_Shape& shape, double deflection);

// Number of faces without a triangulation (each would be a hole).
int CountUnmeshedFaces(const TopoDS_Shape& shape);

// Prints a few facts about the first 'limit' faces without a triangulation.
void DescribeUnmeshedFaces(const std::vector<Body>& bodies, int limit);
