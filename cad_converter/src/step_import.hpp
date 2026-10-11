// =============================================================================
// step_import.hpp — read a STEP file, and list its bodies.
// =============================================================================
#pragma once

#include "body.hpp"

#include <TopoDS_Shape.hxx>

#include <cstddef>
#include <map>
#include <string>
#include <vector>

class TopoDS_TShape;

struct StepModel
{
    TopoDS_Shape shape; // the whole model
    std::map<const TopoDS_TShape*, std::string> partNames; // solid/shell/face of a part -> part name
};

// Reads the STEP file's geometry and assembly structure, with the name of
// every part. Throws std::runtime_error if the file can't be read.
StepModel ReadStep(const std::string& path);

// Every unique body (solid, free shell, loose face) once, unplaced: a part
// placed several times is listed once. 'placements' gets the placed count.
std::vector<TopoDS_Shape> UniqueBodies(const TopoDS_Shape& shape, size_t* placements = nullptr);

// Every placed body, in a stable order: solids, then shells that aren't part
// of a solid, then loose faces, named solid_NNNN, shell_NNNN, face_NNNN.
std::vector<Body> PlacedBodies(const TopoDS_Shape& shape);
