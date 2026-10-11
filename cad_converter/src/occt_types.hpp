// =============================================================================
// occt_types.hpp — short names for the OCCT shape containers.
//
// OCCT 8 deprecates the TopTools_* container names in favour of the
// NCollection templates they stand for; these are the same types.
// =============================================================================
#pragma once

#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_IndexedMap.hxx>
#include <NCollection_List.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS_Shape.hxx>

// Shapes, each once, numbered from 1 (TopExp::MapShapes fills one).
using ShapeIndexMap = NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>;
using ShapeList = NCollection_List<TopoDS_Shape>;
// Each shape with the shapes it belongs to (TopExp::MapShapesAndAncestors).
using ShapeAncestorsMap = NCollection_IndexedDataMap<TopoDS_Shape, ShapeList, TopTools_ShapeMapHasher>;
