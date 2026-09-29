// TessellatedSolid.h
//
// Triangulates an arbitrary OCCT BRep shape (freeform surfaces, fillets,
// anything not cleanly a primitive) into a plain vertex/triangle mesh, and
// can write that mesh out as a standard ASCII STL file.
//
// STL output is unambiguous and needs no guessing -- it's exactly what
// went in this file. Whether/how to get this mesh INTO a Geomega .geo file
// as an inline tessellated-shape block is a separate, currently unverified
// question (see the WriteGeomegaFacetBlock warning below) -- keep those two
// concerns separate rather than assuming the mesh format Geomega expects.

#pragma once

#include <TopoDS_Shape.hxx>
#include <vector>
#include <string>
#include <ostream>

class TessellatedSolid
{
public:
    struct Vertex { double x, y, z; };
    struct Triangle { int v0, v1, v2; }; // indices into m_Vertices

    // linearDeflection / angularDeflection control mesh fidelity (in the
    // same length units as the input shape, i.e. mm for a STEP-sourced
    // shape). Smaller = more triangles, closer to the true surface.
    // 0.1-0.5 mm linear deflection is a reasonable starting point for
    // typical mechanical parts; tighten it for small/curved features.
    explicit TessellatedSolid(const TopoDS_Shape& shape,
                               double linearDeflection = 0.3,
                               double angularDeflection = 0.3);

    // Runs BRepMesh_IncrementalMesh and extracts the resulting
    // triangulation from every face into m_Vertices / m_Triangles.
    // Returns false (and leaves the mesh empty) if meshing failed or the
    // shape had zero triangulatable faces.
    bool Triangulate();

    const std::vector<Vertex>& GetVertices() const { return m_Vertices; }
    const std::vector<Triangle>& GetTriangles() const { return m_Triangles; }

    // Writes a plain ASCII STL file (unit-for-unit what's in the mesh --
    // apply any unit scaling to the shape or the vertices before calling
    // this if you need cm instead of the input's native units).
    bool WriteSTL(const std::string& path, const std::string& solidName) const;

    // EXPERIMENTAL / UNVERIFIED: writes a best-guess inline facet block in
    // a Geomega-like ".Shape.AddVertex / .AddTriangle" style. This syntax
    // has NOT been confirmed against the actual Geomega parser or manual
    // body text (only the manual's table of contents was accessible during
    // development) -- treat this as a starting guess to test against your
    // local `geomega --help` / Geomega.pdf, not as verified output. Prefer
    // WriteSTL() + manually confirming Geomega's real STL/mesh import
    // keyword (if any) over trusting this function's syntax as-is.
    void WriteGeomegaFacetBlock(std::ostream& out,
                                 const std::string& volumeName,
                                 double unitScale) const;

private:
    TopoDS_Shape m_Shape;
    double m_LinearDeflection;
    double m_AngularDeflection;
    std::vector<Vertex> m_Vertices;
    std::vector<Triangle> m_Triangles;
};