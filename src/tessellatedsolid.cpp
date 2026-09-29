// TessellatedSolid.cpp

#include "TessellatedSolid.h"

#include <BRepMesh_IncrementalMesh.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopAbs_Orientation.hxx>
#include <Poly_Triangulation.hxx>
#include <TopLoc_Location.hxx>
#include <BRep_Tool.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <fstream>
#include <iomanip>
#include <cmath>

TessellatedSolid::TessellatedSolid(const TopoDS_Shape& shape,
                                    double linearDeflection,
                                    double angularDeflection)
    : m_Shape(shape)
    , m_LinearDeflection(linearDeflection)
    , m_AngularDeflection(angularDeflection)
{
}

bool TessellatedSolid::Triangulate()
{
    m_Vertices.clear();
    m_Triangles.clear();

    if (m_Shape.IsNull())
        return false;

    // Build the mesh on the shape. isRelative=false means the deflection
    // values are absolute lengths (same units as the shape), not a
    // fraction of the shape's bounding box -- that's usually what you
    // want for consistent quality across parts of very different sizes.
    BRepMesh_IncrementalMesh mesher(m_Shape, m_LinearDeflection, /*isRelative*/ false,
                                     m_AngularDeflection, /*isInParallel*/ true);
    if (!mesher.IsDone())
        return false;

    for (TopExp_Explorer exp(m_Shape, TopAbs_FACE); exp.More(); exp.Next())
    {
        TopoDS_Face face = TopoDS::Face(exp.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull())
            continue; // this face didn't mesh (degenerate/tiny) -- skip it

        const gp_Trsf& trsf = loc.Transformation();
        bool reversed = (face.Orientation() == TopAbs_REVERSED);

        // Record the index offset for this face's vertices within the
        // shape-wide vertex array, since Poly_Triangulation node indices
        // are local to the face.
        int indexOffset = static_cast<int>(m_Vertices.size());

        for (int i = 1; i <= tri->NbNodes(); ++i)
        {
            gp_Pnt p = tri->Node(i);
            p.Transform(trsf);
            m_Vertices.push_back({p.X(), p.Y(), p.Z()});
        }

        for (int i = 1; i <= tri->NbTriangles(); ++i)
        {
            Standard_Integer n1, n2, n3;
            tri->Triangle(i).Get(n1, n2, n3);
            // Poly_Triangulation node indices are 1-based and local to
            // this face; shift to 0-based and offset into the shared
            // vertex array.
            int i1 = indexOffset + (n1 - 1);
            int i2 = indexOffset + (n2 - 1);
            int i3 = indexOffset + (n3 - 1);
            if (reversed)
                m_Triangles.push_back({i1, i3, i2}); // flip winding
            else
                m_Triangles.push_back({i1, i2, i3});
        }
    }

    return !m_Triangles.empty();
}

static TessellatedSolid::Vertex ComputeNormal(const TessellatedSolid::Vertex& a,
                                               const TessellatedSolid::Vertex& b,
                                               const TessellatedSolid::Vertex& c)
{
    double ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
    double vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
    double nx = uy * vz - uz * vy;
    double ny = uz * vx - ux * vz;
    double nz = ux * vy - uy * vx;
    double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len < 1e-12)
        return {0.0, 0.0, 0.0};
    return {nx / len, ny / len, nz / len};
}

bool TessellatedSolid::WriteSTL(const std::string& path, const std::string& solidName) const
{
    if (m_Triangles.empty())
        return false;

    std::ofstream out(path);
    if (!out)
        return false;

    out << "solid " << solidName << "\n";
    out << std::fixed << std::setprecision(6);
    for (const auto& t : m_Triangles)
    {
        const Vertex& a = m_Vertices[t.v0];
        const Vertex& b = m_Vertices[t.v1];
        const Vertex& c = m_Vertices[t.v2];
        Vertex n = ComputeNormal(a, b, c);
        out << "  facet normal " << n.x << " " << n.y << " " << n.z << "\n";
        out << "    outer loop\n";
        out << "      vertex " << a.x << " " << a.y << " " << a.z << "\n";
        out << "      vertex " << b.x << " " << b.y << " " << b.z << "\n";
        out << "      vertex " << c.x << " " << c.y << " " << c.z << "\n";
        out << "    endloop\n";
        out << "  endfacet\n";
    }
    out << "endsolid " << solidName << "\n";
    return true;
}

void TessellatedSolid::WriteGeomegaFacetBlock(std::ostream& out,
                                               const std::string& volumeName,
                                               double unitScale) const
{
    out << "// EXPERIMENTAL / UNVERIFIED inline facet block -- this syntax has\n";
    out << "// NOT been confirmed against Geomega's actual parser. Test it, or\n";
    out << "// replace with whatever real mesh-import keyword you confirm from\n";
    out << "// `geomega --help` or Geomega.pdf before trusting it.\n";
    out << volumeName << ".Shape TESSELATED  // <-- guessed keyword, verify\n";
    for (const auto& v : m_Vertices)
    {
        out << volumeName << ".Shape.AddVertex "
            << v.x * unitScale << " " << v.y * unitScale << " " << v.z * unitScale << "\n";
    }
    for (const auto& t : m_Triangles)
    {
        out << volumeName << ".Shape.AddTriangle " << t.v0 << " " << t.v1 << " " << t.v2 << "\n";
    }
}