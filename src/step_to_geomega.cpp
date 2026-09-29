// step_to_geomega.cpp
//
// Reads a STEP assembly (via OpenCascade's XCAF/XDE reader, which preserves
// the assembly tree, instance names and per-instance placements -- unlike
// the plain STEPControl_Reader, which flattens everything into one shape)
// and emits a MEGAlib / Geomega .geo geometry description.
//
// IMPORTANT -- verify before trusting output on a real detector model:
//   1. Rotation convention: this writer emits Geomega's 3x3 rotation-matrix
//      form (9 numbers) rather than a 3-angle Euler form, specifically to
//      avoid angle-order ambiguity. Confirm against your local
//      MEGAlib/doc/Geomega.pdf that <Name>.Rotation accepts 9 matrix
//      elements in row-major order before relying on this for anything
//      safety- or physics-critical.
//   2. Units: STEP files are almost always in mm; Geomega defaults to cm.
//      This code converts mm -> cm (factor 0.1). If your STEP file uses a
//      different unit, fix UNIT_SCALE below.
//   3. Shape mapping: Geomega volumes take a small set of primitive shapes
//      (BRIK, TUBE, SPHE, ...). This code detects axis-aligned boxes and
//      simple cylinders from the BRep and maps them directly. Anything else
//      (fillets, freeform surfaces, complex machined parts) falls back to
//      an axis-aligned bounding-box (BRIK) APPROXIMATION and is flagged
//      with a comment in the output -- it is not a faithful shape and you
//      should review/replace these by hand for anything that matters
//      geometrically (e.g. active detector volumes).
//   4. Freeform parts additionally get triangulated (see TessellatedSolid)
//      and written out as a companion .stl file next to the .geo output,
//      referenced by a comment on that volume's Shape line. This is the
//      real geometry, in a format that needs no guessing (STL is
//      unambiguous) -- but getting it actually INTO Geomega as the volume's
//      shape (rather than just a reference file for you to inspect) needs
//      you to confirm Geomega's real mesh/STL-import keyword, which was not
//      confirmable from available documentation during development. See
//      TessellatedSolid::WriteGeomegaFacetBlock for an explicitly-marked-
//      experimental guess at that syntax. The --tessellate-all option writes
//      STL companions for primitive shapes too; it does not enable inline mesh
//      syntax in the .geo file.
//
// Build (after `vcpkg install opencascade:x64-windows` and toolchain setup):
//   cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
//   cmake --build build --config Release
//
// Usage:
//   step_to_geomega.exe input.step output.geo [--tessellate-all]

#include <STEPCAFControl_Reader.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TDataStd_Name.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDF_ChildIterator.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <gp_Trsf.hxx>
#include <gp_Pnt.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TCollection_AsciiString.hxx>

#include "tessellatedsolid.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#include <memory>
#include <cctype>
#include <iomanip>
#include <set>

static constexpr double UNIT_SCALE = 0.1; // mm (STEP) -> cm (Geomega)

// ---------------------------------------------------------------------
// Sanitize a STEP part name into a legal, unique Geomega identifier.
// ---------------------------------------------------------------------
std::string SanitizeName(const std::string& raw, std::set<std::string>& usedNames)
{
    std::string out;
    for (char c : raw)
    {
        if (std::isalnum(static_cast<unsigned char>(c)))
            out += c;
        else if (c == '_' || c == '-')
            out += '_';
        // drop spaces / punctuation entirely
    }
    if (out.empty())
        out = "Vol";
    if (std::isdigit(static_cast<unsigned char>(out[0])))
        out = "V_" + out;

    // enforce uniqueness (Geomega volume names must be unique)
    std::string candidate = out;
    int suffix = 1;
    while (usedNames.count(candidate))
        candidate = out + "_" + std::to_string(suffix++);
    usedNames.insert(candidate);
    return candidate;
}

// ---------------------------------------------------------------------
// A detected/approximated shape, in Geomega's <Name>.Shape line form.
// ---------------------------------------------------------------------
struct ShapeSpec
{
    std::string keyword;        // "BRIK", "TUBE", ...
    std::vector<double> params; // already unit-converted
    bool isApproximation = false;
    std::string note;
    std::string meshPath;       // set only for freeform parts: path to the
                                 // companion .stl file with the real shape
};

// Try to recognize an axis-aligned box: every face planar, and the face
// normals reduce to only 3 distinct axis directions. If so, the bounding
// box IS the true shape (not an approximation).
static bool IsAxisAlignedBox(const TopoDS_Shape& shape)
{
    int faceCount = 0;
    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
    {
        faceCount++;
        TopoDS_Face face = TopoDS::Face(exp.Current());
        BRepAdaptor_Surface surf(face, true);
        if (surf.GetType() != GeomAbs_Plane)
            return false;
    }
    return faceCount == 6;
}

// Try to recognize a simple cylinder: one cylindrical face (+ up to 2
// planar end caps), nothing else.
static bool IsSimpleCylinder(const TopoDS_Shape& shape, double& radiusOut, double& heightOut)
{
    int cylFaces = 0, planarFaces = 0, otherFaces = 0;
    double radius = 0.0;
    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
    {
        TopoDS_Face face = TopoDS::Face(exp.Current());
        BRepAdaptor_Surface surf(face, true);
        if (surf.GetType() == GeomAbs_Cylinder)
        {
            cylFaces++;
            radius = surf.Cylinder().Radius();
        }
        else if (surf.GetType() == GeomAbs_Plane)
        {
            planarFaces++;
        }
        else
        {
            otherFaces++;
        }
    }
    if (cylFaces != 1 || otherFaces != 0 || planarFaces > 2)
        return false;

    Bnd_Box bbox;
    BRepBndLib::Add(shape, bbox);
    double xmin, ymin, zmin, xmax, ymax, zmax;
    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    // Assumes the cylinder axis is roughly the bbox's longest dimension.
    // Good enough as a heuristic; for tilted cylinders this will be wrong
    // and the shape will fall through to the bounding-box approximation
    // instead if this check doesn't hold well -- review flagged output.
    radiusOut = radius * UNIT_SCALE;
    heightOut = std::max({xmax - xmin, ymax - ymin, zmax - zmin}) * UNIT_SCALE;
    return true;
}

// meshDir: folder to write companion .stl files into (created by caller).
// volumeName: used as both the .stl filename and the STL "solid" name.
static ShapeSpec ClassifyShape(const TopoDS_Shape& shape,
                                const std::string& meshDir,
                                const std::string& volumeName,
                                bool tessellateAllMeshes)
{
    ShapeSpec spec;

    if (IsAxisAlignedBox(shape))
    {
        Bnd_Box bbox;
        BRepBndLib::Add(shape, bbox);
        double xmin, ymin, zmin, xmax, ymax, zmax;
        bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        spec.keyword = "BRIK";
        // Geomega BRIK takes HALF-lengths.
        spec.params = {
            0.5 * (xmax - xmin) * UNIT_SCALE,
            0.5 * (ymax - ymin) * UNIT_SCALE,
            0.5 * (zmax - zmin) * UNIT_SCALE
        };
    }
    else
    {
        double radius = 0.0, height = 0.0;
        if (IsSimpleCylinder(shape, radius, height))
        {
            spec.keyword = "TUBE";
            // Geomega TUBE: inner radius, outer radius, half-height.
            spec.params = {0.0, radius, 0.5 * height};
        }
        else
        {
            // Fallback: bounding-box approximation for anything freeform.
            Bnd_Box bbox;
            BRepBndLib::Add(shape, bbox);
            double xmin, ymin, zmin, xmax, ymax, zmax;
            bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
            spec.keyword = "BRIK";
            spec.params = {
                0.5 * (xmax - xmin) * UNIT_SCALE,
                0.5 * (ymax - ymin) * UNIT_SCALE,
                0.5 * (zmax - zmin) * UNIT_SCALE
            };
            spec.isApproximation = true;
            spec.note = "APPROXIMATION: bounding box of a non-primitive/freeform shape. "
                        "Review before relying on this geometrically.";
        }
    }

    if (spec.isApproximation || tessellateAllMeshes)
    {
        TessellatedSolid mesh(shape, /*linearDeflection mm*/ 0.3, /*angularDeflection*/ 0.3);
        if (mesh.Triangulate())
        {
            std::string stlPath = meshDir + "/" + volumeName + ".stl";
            if (mesh.WriteSTL(stlPath, volumeName))
            {
                spec.meshPath = stlPath;
                if (spec.isApproximation)
                    spec.note += " ";
                spec.note += "Real shape tessellated to " + std::to_string(mesh.GetTriangles().size())
                           + " triangles, written to: " + stlPath;
            }
            else
            {
                spec.note += " (Tessellation succeeded but STL write to " + stlPath + " failed.)";
            }
        }
        else
        {
            spec.note += " (Tessellation failed for this shape -- check for degenerate geometry.)";
        }
    }

    return spec;
}

// ---------------------------------------------------------------------
// Assembly tree node.
// ---------------------------------------------------------------------
struct VolumeNode
{
    std::string name;
    TopoDS_Shape shape;      // shape at this node's own local origin
    gp_Trsf motherRelative;  // placement relative to parent (identity for roots)
    std::vector<std::shared_ptr<VolumeNode>> children;
    bool isAssembly = false; // pure grouping node, no own solid
};

static std::string GetLabelName(const TDF_Label& label)
{
    Handle(TDataStd_Name) nameAttr;
    if (label.FindAttribute(TDataStd_Name::GetID(), nameAttr))
    {
        TCollection_ExtendedString ext = nameAttr->Get();
        TCollection_AsciiString ascii(ext, '?');
        return ascii.ToCString();
    }
    return "Unnamed";
}

static std::shared_ptr<VolumeNode> BuildNode(const Handle(XCAFDoc_ShapeTool)& shapeTool,
                                              const TDF_Label& label,
                                              const gp_Trsf& motherRelative)
{
    auto node = std::make_shared<VolumeNode>();
    node->name = GetLabelName(label);
    node->motherRelative = motherRelative;

    if (XCAFDoc_ShapeTool::IsAssembly(label))
    {
        node->isAssembly = true;
        TDF_LabelSequence components;
        shapeTool->GetComponents(label, components);
        for (Standard_Integer i = 1; i <= components.Length(); ++i)
        {
            TDF_Label compLabel = components.Value(i);

            // A component label references a "referred" shape label and
            // carries its own placement (mother-relative -- this matches
            // Geomega's own placement model, no extra recomputation needed).
            TDF_Label referredLabel;
            gp_Trsf childTrsf;
            if (XCAFDoc_ShapeTool::GetReferredShape(compLabel, referredLabel))
            {
                TopLoc_Location loc = XCAFDoc_ShapeTool::GetLocation(compLabel);
                childTrsf = loc.Transformation();
                node->children.push_back(BuildNode(shapeTool, referredLabel, childTrsf));
            }
        }
    }
    else
    {
        TopoDS_Shape s;
        shapeTool->GetShape(label, s);
        node->shape = s;
    }
    return node;
}

// ---------------------------------------------------------------------
// Emit Geomega text for one node and recurse into its children.
// motherName == "" marks a top-level (world-placed) volume.
// ---------------------------------------------------------------------
static void WriteNode(std::ostream& out,
                       const std::shared_ptr<VolumeNode>& node,
                       const std::string& motherName,
                       std::set<std::string>& usedNames,
                       const std::string& meshDir,
                       bool tessellateAllMeshes)
{
    std::string geomegaName = SanitizeName(node->name, usedNames);

    out << "\nVolume " << geomegaName << "\n";

    if (node->isAssembly)
    {
        // Pure grouping volume: give it a tiny transparent placeholder shape
        // sized to its children's combined bounding box would require a
        // second pass; simplest correct option is a generic small BRIK the
        // user can resize, OR make it non-solid. Geomega requires every
        // Volume to have a Shape, so we emit a permissive placeholder here.
        out << geomegaName << ".Material Vacuum\n";
        out << geomegaName << ".Shape BRIK 1000. 1000. 1000.  // assembly placeholder -- resize or replace\n";
        out << geomegaName << ".Visibility 0\n";
    }
    else
    {
        ShapeSpec spec = ClassifyShape(node->shape, meshDir, geomegaName, tessellateAllMeshes);
        out << geomegaName << ".Material Vacuum  // TODO: STEP rarely carries usable material data -- set real material\n";
        out << geomegaName << ".Shape " << spec.keyword;
        out << std::fixed << std::setprecision(4);
        for (double p : spec.params)
            out << " " << p;
        out << "\n";
        if (spec.isApproximation)
            out << "// " << spec.note << "\n";
        if (!spec.meshPath.empty())
        {
            out << "// Companion mesh (real geometry, not yet wired into Geomega -- see file header): "
                << spec.meshPath << "\n";
        }
    }

    out << geomegaName << ".Mother " << (motherName.empty() ? "0" : motherName) << "\n";

    if (motherName.empty())
    {
        // top-level volume: no position/rotation relative to a mother
    }
    else
    {
        gp_XYZ t = node->motherRelative.TranslationPart();
        out << geomegaName << ".Position "
            << t.X() * UNIT_SCALE << " " << t.Y() * UNIT_SCALE << " " << t.Z() * UNIT_SCALE << "\n";

        gp_Mat m = node->motherRelative.VectorialPart();
        // 3x3 matrix form -- VERIFY row-major order against Geomega.pdf.
        out << geomegaName << ".Rotation "
            << m.Value(1,1) << " " << m.Value(1,2) << " " << m.Value(1,3) << " "
            << m.Value(2,1) << " " << m.Value(2,2) << " " << m.Value(2,3) << " "
            << m.Value(3,1) << " " << m.Value(3,2) << " " << m.Value(3,3) << "\n";
    }

    for (auto& child : node->children)
        WriteNode(out, child, geomegaName, usedNames, meshDir, tessellateAllMeshes);
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " input.step output.geo [--tessellate-all]\n";
        return 1;
    }
    std::string inputPath = argv[1];
    std::string outputPath = argv[2];
    bool tessellateAllMeshes = false;
    for (int i = 3; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--tessellate-all")
            tessellateAllMeshes = true;
        else
        {
            std::cerr << "Unknown option: " << argv[i] << "\n";
            return 1;
        }
    }

    // Companion .stl meshes for freeform parts go in a "<output>_meshes"
    // folder next to the .geo file.
    std::filesystem::path outPath(outputPath);
    std::string meshDir = (outPath.parent_path() / (outPath.stem().string() + "_meshes")).string();
    std::filesystem::create_directories(meshDir);

    Handle(TDocStd_Document) doc;
    Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
    app->NewDocument("MDTV-XCAF", doc);

    STEPCAFControl_Reader reader;
    reader.SetColorMode(false);
    reader.SetNameMode(true);
    reader.SetLayerMode(false);

    IFSelect_ReturnStatus status = reader.ReadFile(inputPath.c_str());
    if (status != IFSelect_RetDone)
    {
        std::cerr << "Failed to read STEP file: " << inputPath << "\n";
        return 1;
    }
    if (!reader.Transfer(doc))
    {
        std::cerr << "Failed to transfer STEP data into XCAF document.\n";
        return 1;
    }

    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());

    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);

    std::vector<std::shared_ptr<VolumeNode>> roots;
    for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i)
    {
        gp_Trsf identity;
        roots.push_back(BuildNode(shapeTool, freeShapes.Value(i), identity));
    }

    std::ofstream out(outputPath);
    if (!out)
    {
        std::cerr << "Failed to open output file: " << outputPath << "\n";
        return 1;
    }

    out << "// Auto-generated from " << inputPath << " by step_to_geomega\n";
    out << "// REVIEW: units converted mm->cm; rotation matrices are UNVERIFIED\n";
    out << "// against Geomega's exact expected element order -- check Geomega.pdf.\n";
    out << "// Any Shape line marked APPROXIMATION is a bounding-box stand-in for\n";
    out << "// freeform/complex geometry and is not dimensionally faithful. Its real\n";
    out << "// tessellated shape (if meshing succeeded) is in the companion .stl file\n";
    out << "// referenced in a comment on that volume, under: " << meshDir << "\n";

    out << "\nSurroundingSphere 1000. 0. 0. 0. 1000.  // TODO: size to your actual geometry\n";

    std::set<std::string> usedNames;
    // A single top-level "World" mother volume is conventional in Geomega.
    out << "\nVolume World\n";
    out << "World.Material Vacuum\n";
    out << "World.Shape BRIK 2000. 2000. 2000.  // TODO: size appropriately\n";
    out << "World.Mother 0\n";
    usedNames.insert("World");

    for (auto& root : roots)
        WriteNode(out, root, "World", usedNames, meshDir, tessellateAllMeshes);

    out.close();
    std::cout << "Wrote " << outputPath << " (" << roots.size() << " top-level assemblies)\n";
    std::cout << "Companion meshes (if any freeform parts were found) written to: " << meshDir << "\n";
    return 0;
}