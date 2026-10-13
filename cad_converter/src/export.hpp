// =============================================================================
// export.hpp — group the bodies into named objects and write them to STL or
// OBJ.
// =============================================================================
#pragma once

#include "body.hpp"
#include "weld.hpp"

#include <cstddef>
#include <string>
#include <vector>

struct OutputObject
{
    std::string name;
    std::vector<size_t> bodies; // indices into the body list
};

// One object per part (all its placements), or with 'perBody' one per body.
// Bodies whose mesh is empty (dropped) are left out. Naming: see export.cpp.
std::vector<OutputObject> GroupBodies(const std::vector<Body>& bodies, const std::vector<SurfaceMesh>& meshes,
                                      bool perBody);

// Multi-solid ASCII STL: one "solid <name> ... endsolid <name>" block per object.
void ExportStl(const std::vector<OutputObject>& objects, const std::vector<SurfaceMesh>& meshes,
               const std::string& outputPath);

// Wavefront OBJ: one "o <name>" object per object, with indexed vertices.
void ExportObj(const std::vector<OutputObject>& objects, const std::vector<SurfaceMesh>& meshes,
               const std::string& outputPath);
