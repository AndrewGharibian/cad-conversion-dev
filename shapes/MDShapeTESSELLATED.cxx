#include "MDShapeTESSELLATED.h"

#include <TGeoTessellated.h>

#include <algorithm>
#include <cmath>
#include <sstream>

#ifdef ___CLING___
ClassImp(MDShapeTESSELLATED)
#endif

MDShapeTESSELLATED::MDShapeTESSELLATED(const MString& Name) : MDShape(Name)
{
    m_Type = "TESSELLATED";
}

MDShapeTESSELLATED::~MDShapeTESSELLATED()
{
}

bool MDShapeTESSELLATED::Set(const std::vector<Vertex>& Vertices,
                             const std::vector<Triangle>& Triangles)
{
    if (Vertices.size() < 4 || Triangles.size() < 4)
        return false;

    for (const Vertex& Vertex : Vertices)
    {
        if (!std::isfinite(Vertex.x) || !std::isfinite(Vertex.y) || !std::isfinite(Vertex.z))
            return false;
    }

    for (const Triangle& Triangle : Triangles)
    {
        if (Triangle.v0 < 0 || Triangle.v1 < 0 || Triangle.v2 < 0 ||
            Triangle.v0 >= static_cast<int>(Vertices.size()) ||
            Triangle.v1 >= static_cast<int>(Vertices.size()) ||
            Triangle.v2 >= static_cast<int>(Vertices.size()) ||
            Triangle.v0 == Triangle.v1 || Triangle.v1 == Triangle.v2 || Triangle.v2 == Triangle.v0)
            return false;
    }

    m_Vertices = Vertices;
    m_Triangles = Triangles;
    m_IsValidated = false;
    return true;
}

bool MDShapeTESSELLATED::Validate()
{
    if (m_IsValidated)
        return true;

    if (m_Vertices.size() < 4 || m_Triangles.size() < 4)
        return false;

    TGeoTessellated* Geo = new TGeoTessellated(m_Name.Data());
    for (const Triangle& Triangle : m_Triangles)
    {
        const Vertex& A = m_Vertices[Triangle.v0];
        const Vertex& B = m_Vertices[Triangle.v1];
        const Vertex& C = m_Vertices[Triangle.v2];
        const TGeoTessellated::Vertex_t RootA{A.x, A.y, A.z};
        const TGeoTessellated::Vertex_t RootB{B.x, B.y, B.z};
        const TGeoTessellated::Vertex_t RootC{C.x, C.y, C.z};
        if (!Geo->AddFacet(RootA, RootB, RootC))
        {
            delete Geo;
            return false;
        }
    }

    Geo->CloseShape(true, true, false);
    if (!Geo->IsClosedBody())
    {
        delete Geo;
        return false;
    }

    delete m_Geo;
    m_Geo = Geo;
    m_IsValidated = true;
    return true;
}

bool MDShapeTESSELLATED::Parse(const MTokenizer& Tokenizer, const MDDebugInfo& Info)
{
    if (Tokenizer.GetNTokens() < 4 || !Tokenizer.IsTokenAt(1, "Shape") ||
        !Tokenizer.IsTokenAt(2, "TESSELLATED"))
    {
        Info.Error("Expected <Volume>.Shape TESSELLATED <vertex-count> ...");
        return false;
    }

    const int VertexCount = Tokenizer.GetTokenAtAsInt(3);
    if (VertexCount < 4)
    {
        Info.Error("A tessellated solid requires at least four vertices.");
        return false;
    }

    const unsigned int TriangleCountIndex = 4 + 3 * static_cast<unsigned int>(VertexCount);
    if (TriangleCountIndex >= Tokenizer.GetNTokens())
    {
        Info.Error("The tessellated shape ended before its triangle count.");
        return false;
    }

    const int TriangleCount = Tokenizer.GetTokenAtAsInt(TriangleCountIndex);
    if (TriangleCount < 4)
    {
        Info.Error("A closed tessellated solid requires at least four triangles.");
        return false;
    }

    const unsigned int ExpectedTokenCount = TriangleCountIndex + 1 + 3 * static_cast<unsigned int>(TriangleCount);
    if (Tokenizer.GetNTokens() != ExpectedTokenCount)
    {
        Info.Error("The tessellated shape vertex or triangle data has an incorrect length.");
        return false;
    }

    std::vector<Vertex> Vertices;
    Vertices.reserve(VertexCount);
    for (int Index = 0; Index < VertexCount; ++Index)
    {
        const unsigned int TokenIndex = 4 + 3 * static_cast<unsigned int>(Index);
        Vertices.push_back({Tokenizer.GetTokenAtAsDouble(TokenIndex),
                            Tokenizer.GetTokenAtAsDouble(TokenIndex + 1),
                            Tokenizer.GetTokenAtAsDouble(TokenIndex + 2)});
    }

    std::vector<Triangle> Triangles;
    Triangles.reserve(TriangleCount);
    const unsigned int FirstTriangleIndex = TriangleCountIndex + 1;
    for (int Index = 0; Index < TriangleCount; ++Index)
    {
        const unsigned int TokenIndex = FirstTriangleIndex + 3 * static_cast<unsigned int>(Index);
        Triangles.push_back({Tokenizer.GetTokenAtAsInt(TokenIndex),
                             Tokenizer.GetTokenAtAsInt(TokenIndex + 1),
                             Tokenizer.GetTokenAtAsInt(TokenIndex + 2)});
    }

    if (!Set(Vertices, Triangles))
    {
        Info.Error("The tessellated shape contains invalid coordinates or triangle indices.");
        return false;
    }

    return true;
}

MVector MDShapeTESSELLATED::GetSize()
{
    if (m_Vertices.empty())
        return MVector(0, 0, 0);

    double Xmin = m_Vertices.front().x;
    double Ymin = m_Vertices.front().y;
    double Zmin = m_Vertices.front().z;
    double Xmax = Xmin;
    double Ymax = Ymin;
    double Zmax = Zmin;
    for (const Vertex& Vertex : m_Vertices)
    {
        Xmin = std::min(Xmin, Vertex.x);
        Ymin = std::min(Ymin, Vertex.y);
        Zmin = std::min(Zmin, Vertex.z);
        Xmax = std::max(Xmax, Vertex.x);
        Ymax = std::max(Ymax, Vertex.y);
        Zmax = std::max(Zmax, Vertex.z);
    }

    return MVector(0.5 * (Xmax - Xmin), 0.5 * (Ymax - Ymin), 0.5 * (Zmax - Zmin));
}

MString MDShapeTESSELLATED::ToString()
{
    std::ostringstream Out;
    Out << "TESSELLATED (" << m_Vertices.size() << " vertices, "
        << m_Triangles.size() << " triangles)";
    return Out.str().c_str();
}

MString MDShapeTESSELLATED::GetGeomega() const
{
    std::ostringstream Out;
    Out << "TESSELLATED " << m_Vertices.size();
    for (const Vertex& Vertex : m_Vertices)
        Out << " " << Vertex.x << " " << Vertex.y << " " << Vertex.z;

    Out << " " << m_Triangles.size();
    for (const Triangle& Triangle : m_Triangles)
        Out << " " << Triangle.v0 << " " << Triangle.v1 << " " << Triangle.v2;

    return Out.str().c_str();
}

double MDShapeTESSELLATED::GetVolume()
{
    double SignedVolume = 0.0;
    for (const Triangle& Triangle : m_Triangles)
    {
        const Vertex& A = m_Vertices[Triangle.v0];
        const Vertex& B = m_Vertices[Triangle.v1];
        const Vertex& C = m_Vertices[Triangle.v2];
        SignedVolume += A.x * (B.y * C.z - B.z * C.y) +
                        A.y * (B.z * C.x - B.x * C.z) +
                        A.z * (B.x * C.y - B.y * C.x);
    }
    return std::abs(SignedVolume / 6.0);
}

bool MDShapeTESSELLATED::Scale(const double Scaler, const MString Axes)
{
    if (Scaler == m_Scaler && Axes == m_ScalingAxis)
        return true;
    if (!MDShape::Scale(Scaler, Axes))
        return false;
    if (!IsScaled())
        return true;

    for (Vertex& Vertex : m_Vertices)
    {
        if (m_ScalingAxis.Contains("X")) Vertex.x *= m_Scaler;
        if (m_ScalingAxis.Contains("Y")) Vertex.y *= m_Scaler;
        if (m_ScalingAxis.Contains("Z")) Vertex.z *= m_Scaler;
    }
    m_IsValidated = false;
    return Validate();
}

MVector MDShapeTESSELLATED::GetUniquePosition() const
{
    if (m_Vertices.empty())
        return MVector(0, 0, 0);

    double Xmin = m_Vertices.front().x;
    double Ymin = m_Vertices.front().y;
    double Zmin = m_Vertices.front().z;
    double Xmax = Xmin;
    double Ymax = Ymin;
    double Zmax = Zmin;
    for (const Vertex& Vertex : m_Vertices)
    {
        Xmin = std::min(Xmin, Vertex.x);
        Ymin = std::min(Ymin, Vertex.y);
        Zmin = std::min(Zmin, Vertex.z);
        Xmax = std::max(Xmax, Vertex.x);
        Ymax = std::max(Ymax, Vertex.y);
        Zmax = std::max(Zmax, Vertex.z);
    }

    return MVector(0.5 * (Xmin + Xmax), 0.5 * (Ymin + Ymax), 0.5 * (Zmin + Zmax));
}