// =============================================================================
// step_import.hpp — read a STEP file, and list its bodies (its solids).
// =============================================================================
#pragma once

#include "body.hpp"

#include <TopoDS_Shape.hxx>

#include <cstddef>
#include <map>
#include <string>
#include <vector>

class TopoDS_TShape;

// A part (STEP product) that bodies are placements of.
struct PartInfo
{
    std::string name;
    size_t bodies = 1; // solids in one placement of the part (a part can hold several)
};

struct StepModel
{
    TopoDS_Shape shape; // the whole model: a compound of its placed solids
    std::map<const TopoDS_TShape*, PartInfo> parts; // solid of a part -> its part
};

// Reads the STEP file's geometry and assembly structure, with the name of
// every part. Only solids are kept: shells and loose faces are removed.
// Throws std::runtime_error if the file can't be read.
StepModel ReadStep(const std::string& path);

// True if 'solid' is a solid whose shells are all closed.
bool IsWatertight(const TopoDS_Shape& solid);

// Every unique solid once, unplaced: a part placed several times is listed
// once. 'placements' gets the placed count.
std::vector<TopoDS_Shape> UniqueBodies(const TopoDS_Shape& shape, size_t* placements = nullptr);

// Every placed solid, in a stable order, named solid_NNNN.
std::vector<Body> PlacedBodies(const TopoDS_Shape& shape);

// "solid_NNNN" for the body at 'index' (from 0).
std::string SolidName(size_t index);
