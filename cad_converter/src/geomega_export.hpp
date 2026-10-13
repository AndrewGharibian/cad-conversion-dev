// =============================================================================
// geomega_export.hpp — write the objects as a MEGAlib/Geomega geometry:
// one TESSELLATED volume per body, in one .geo file per object, included by a
// .geo.setup file where the materials are set.
// =============================================================================
#pragma once

#include "export.hpp"

#include <string>
#include <vector>

struct GeomegaReport
{
    std::string meshDir; // folder of the per-object .geo files
    size_t volumes = 0;
};

// Writes 'setupPath' (<model>.geo.setup) and <model>_geo/<object>.geo next to
// it. 'source' names the input file in the header comments.
GeomegaReport ExportGeomega(const std::vector<OutputObject>& objects, const std::vector<SurfaceMesh>& meshes,
                            const std::string& setupPath, const std::string& source);
