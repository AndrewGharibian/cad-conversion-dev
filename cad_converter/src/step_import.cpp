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
// The whole model comes back as one shape: the top-level (free) shape, or a
// compound of them if there are several.
// =============================================================================
#include "step_import.hpp"

#include "stage_timer.hpp"

#include <BRep_Builder.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <NCollection_Sequence.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <TCollection_AsciiString.hxx>
#include <TDF_Label.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Compound.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <iomanip>
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

    // Every part (a label with a shape of its own, not an assembly), by the
    // bodies in its shape.
    NCollection_Sequence<TDF_Label> parts;
    shapes->GetShapes(parts);
    for (const TDF_Label& label : parts)
    {
        Handle(TDataStd_Name) name;
        if (shapes->IsAssembly(label) || !label.FindAttribute(TDataStd_Name::GetID(), name))
        {
            continue;
        }
        const std::string partName = TCollection_AsciiString(name->Get()).ToCString();
        const TopoDS_Shape partShape = shapes->GetShape(label);
        for (TopAbs_ShapeEnum type : {TopAbs_SOLID, TopAbs_SHELL, TopAbs_FACE})
        {
            for (TopExp_Explorer exp(partShape, type); exp.More(); exp.Next())
            {
                model.partNames.emplace(exp.Current().TShape().get(), partName);
            }
        }
    }
    return model;
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
    for (TopExp_Explorer exp(shape, TopAbs_SHELL, TopAbs_SOLID); exp.More(); exp.Next())
    {
        add(exp.Current());
    }
    for (TopExp_Explorer exp(shape, TopAbs_FACE, TopAbs_SHELL); exp.More(); exp.Next())
    {
        add(exp.Current());
    }
    if (placements)
    {
        *placements = count;
    }
    return bodies;
}

// UniqueBodies collapses repeated placements; this keeps each one, since each
// is its own volume.
std::vector<Body> PlacedBodies(const TopoDS_Shape& shape)
{
    std::vector<Body> bodies;
    auto add = [&](const TopoDS_Shape& s, const char* kind) {
        std::ostringstream name;
        name << kind << "_" << std::setw(4) << std::setfill('0') << bodies.size() + 1;
        bodies.push_back({name.str(), s, s.ShapeType() == TopAbs_SOLID});
    };
    for (TopExp_Explorer exp(shape, TopAbs_SOLID); exp.More(); exp.Next())
    {
        add(exp.Current(), "solid");
    }
    for (TopExp_Explorer exp(shape, TopAbs_SHELL, TopAbs_SOLID); exp.More(); exp.Next())
    {
        add(exp.Current(), "shell");
    }
    for (TopExp_Explorer exp(shape, TopAbs_FACE, TopAbs_SHELL); exp.More(); exp.Next())
    {
        add(exp.Current(), "face");
    }
    return bodies;
}
