// =============================================================================
// step_import.cpp — read a STEP file, with the name of every part, and list
// its bodies.
//
// STEPCAFControl_Reader parses the STEP file's product structure and geometry
// into an XCAF document: OCCT's native BRep shapes plus the assembly tree,
// where every part ("product" in STEP) is a label carrying its name. A part
// placed several times is one label (one shape) referenced by several
// placements, which share its TopoDS_TShape at different locations. So the
// part a placed solid belongs to is found by its TShape.
//
// The model comes back as one compound of its placed solids. Everything else
// (shells that aren't part of a solid, loose faces) is removed here, before
// any conversion: Geant4 and ROOT need every volume to be a closed,
// watertight mesh, and a surface encloses no volume. (In cosi_test.step these
// are the PCB copper layers, exported as zero-thickness faces.) A solid whose
// own shell is open is removed later, after repair (IsWatertight).
// =============================================================================
#include "step_import.hpp"

#include "stage_timer.hpp"

#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <NCollection_Sequence.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <TCollection_AsciiString.hxx>
#include <TDF_Label.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

StepModel ReadStep(const std::string& path)
{
    STEPCAFControl_Reader reader;
    reader.SetNameMode(true); // part names; colours and layers aren't needed
    reader.SetColorMode(false);
    reader.SetLayerMode(false);

    // Two separately timed phases: parsing the text into STEP entities,
    // then translating those entities into OCCT geometry and topology.
    {
        StageTimer timer("parsing STEP file");
        IFSelect_ReturnStatus status = reader.ReadFile(path.c_str());
        if (status != IFSelect_RetDone)
        {
            throw std::runtime_error("Failed to read STEP file: " + path);
        }
    }

    Handle(TDocStd_Document) document;
    XCAFApp_Application::GetApplication()->NewDocument("MDTV-XCAF", document);
    {
        StageTimer timer("translating STEP entities to B-rep");
        if (!reader.Transfer(document))
        {
            throw std::runtime_error("No shapes could be transferred from: " + path);
        }
    }

    StepModel model;
    Handle(XCAFDoc_ShapeTool) shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
    NCollection_Sequence<TDF_Label> roots;
    shapes->GetFreeShapes(roots);
    if (roots.Length() == 1)
    {
        model.shape = shapes->GetShape(roots.Value(1));
    }
    else
    {
        TopoDS_Compound compound;
        BRep_Builder builder;
        builder.MakeCompound(compound);
        for (const TDF_Label& root : roots)
        {
            builder.Add(compound, shapes->GetShape(root));
        }
        model.shape = compound;
    }
    if (model.shape.IsNull())
    {
        throw std::runtime_error("No shapes could be transferred from: " + path);
    }

    // Keep only the placed solids. Placements still share their part's
    // TopoDS_TShape, which the stages below rely on.
    size_t shells = 0, faces = 0;
    for (TopExp_Explorer exp(model.shape, TopAbs_SHELL, TopAbs_SOLID); exp.More(); exp.Next())
    {
        ++shells;
    }
    for (TopExp_Explorer exp(model.shape, TopAbs_FACE, TopAbs_SHELL); exp.More(); exp.Next())
    {
        ++faces;
    }
    TopoDS_Compound solids;
    BRep_Builder builder;
    builder.MakeCompound(solids);
    for (TopExp_Explorer exp(model.shape, TopAbs_SOLID); exp.More(); exp.Next())
    {
        builder.Add(solids, exp.Current());
    }
    model.shape = solids;
    if (shells + faces > 0)
    {
        std::cout << "  removed " << shells << " shell(s) and " << faces
                  << " loose face(s): surfaces, not watertight solids\n";
    }

    // Every part (a label with a shape of its own, not an assembly), by the
    // solids in its shape. A part without solids isn't written at all.
    NCollection_Sequence<TDF_Label> parts;
    shapes->GetShapes(parts);
    for (const TDF_Label& label : parts)
    {
        Handle(TDataStd_Name) name;
        if (shapes->IsAssembly(label) || !label.FindAttribute(TDataStd_Name::GetID(), name))
        {
            continue;
        }
        const TopoDS_Shape partShape = shapes->GetShape(label);
        const PartInfo part{TCollection_AsciiString(name->Get()).ToCString(), PlacedBodies(partShape).size()};
        for (TopExp_Explorer exp(partShape, TopAbs_SOLID); exp.More(); exp.Next())
        {
            model.parts.emplace(exp.Current().TShape().get(), part);
        }
    }
    return model;
}

// A solid is watertight if it has a shell and every shell (the outer one and
// any voids) is closed: each edge bounds exactly two of its faces.
bool IsWatertight(const TopoDS_Shape& solid)
{
    if (solid.ShapeType() != TopAbs_SOLID)
    {
        return false;
    }
    bool hasShell = false;
    for (TopExp_Explorer exp(solid, TopAbs_SHELL); exp.More(); exp.Next())
    {
        hasShell = true;
        if (!BRep_Tool::IsClosed(TopoDS::Shell(exp.Current())))
        {
            return false;
        }
    }
    return hasShell;
}

// A part placed several times shares one TopoDS_TShape at different
// locations, so unique bodies are told apart by TShape.
std::vector<TopoDS_Shape> UniqueBodies(const TopoDS_Shape& shape, size_t* placements)
{
    std::vector<TopoDS_Shape> bodies;
    std::map<const TopoDS_TShape*, bool> seen;
    size_t count = 0;
    auto add = [&](const TopoDS_Shape& body) {
        ++count;
        if (seen.emplace(body.TShape().get(), true).second)
        {
            bodies.push_back(body.Located(TopLoc_Location())); // the part itself, unplaced
        }
    };
    for (TopExp_Explorer exp(shape, TopAbs_SOLID); exp.More(); exp.Next())
    {
        add(exp.Current());
    }
    if (placements)
    {
        *placements = count;
    }
    return bodies;
}

std::string SolidName(size_t index)
{
    std::ostringstream name;
    name << "solid_" << std::setw(4) << std::setfill('0') << index + 1;
    return name.str();
}

// UniqueBodies collapses repeated placements; this keeps each one, since each
// is its own volume.
std::vector<Body> PlacedBodies(const TopoDS_Shape& shape)
{
    std::vector<Body> bodies;
    for (TopExp_Explorer exp(shape, TopAbs_SOLID); exp.More(); exp.Next())
    {
        bodies.push_back({SolidName(bodies.size()), exp.Current()});
    }
    return bodies;
}
