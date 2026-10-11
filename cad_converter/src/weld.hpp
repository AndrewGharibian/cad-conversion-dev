// =============================================================================
// weld.hpp — turn OCCT's per-face triangulation into one closed mesh per
// body, and measure it against the exact CAD geometry.
// =============================================================================
#pragma once

#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

// A plain indexed triangle mesh: shared vertices, triangles wound
// counter-clockwise when seen from outside the solid.
struct SurfaceMesh
{
    std::vector<gp_Pnt> vertices;
    std::vector<std::array<int, 3>> triangles;
};

// Quality numbers for a mesh, measured against the exact B-rep.
struct MeshQuality
{
    size_t vertices = 0;
    size_t triangles = 0;
    double maxDeviation = 0.0;      // worst distance from the exact surface
    double minAngleDeg = 180.0;     // smallest triangle angle anywhere
    size_t trianglesBelow20Deg = 0; // triangles with a corner sharper than 20 degrees
    size_t openEdges = 0;           // edges used by 1 triangle (holes/cracks)
    size_t nonManifoldEdges = 0;    // edges used by more than 2 triangles
    size_t deviationEligible = 0;   // triangles whose deviation needed measuring
    size_t deviationMeasured = 0;   // ...of which were measured (< eligible if sampled)
    std::vector<size_t> openEdgesPerBody; // open edges of each body (0 = closed surface)
    // Where the problems are: a few human-readable examples (worst deviation,
    // open edges, edges shared by more than two triangles), for diagnosis.
    std::vector<std::string> notes;
};

// Every body must carry an OCCT triangulation on its faces (faces without one
// leave holes, which the measurement reports). Each body is welded on its own
// (bodies never share vertices, even where they touch) and comes back as its
// own mesh, in the same order; 'quality' gets the measurement. The deviation
// is measured on an even sample of 'deviationSampleLimit' triangles if there
// are more (0 = measure all). Returns an empty vector if there are no
// triangles at all.
std::vector<SurfaceMesh> WeldAndMeasure(const std::vector<TopoDS_Shape>& bodies, size_t deviationSampleLimit,
                                        MeshQuality& quality);
