#pragma once

#include "MDShape.h"

#include <vector>

class MDShapeTESSELLATED : public MDShape
{
public:
    struct Vertex
    {
        double x;
        double y;
        double z;
    };

    struct Triangle
    {
        int v0;
        int v1;
        int v2;
    };

    explicit MDShapeTESSELLATED(const MString& Name);
    virtual ~MDShapeTESSELLATED();

    bool Set(const std::vector<Vertex>& Vertices,
             const std::vector<Triangle>& Triangles);
    bool Validate();
    bool Parse(const MTokenizer& Tokenizer, const MDDebugInfo& Info);

    MVector GetSize();
    MString ToString();
    MString GetGeomega() const;
    double GetVolume();
    virtual bool Scale(const double Scaler, const MString Axes = "XYZ");
    virtual MVector GetUniquePosition() const;

private:
    std::vector<Vertex> m_Vertices;
    std::vector<Triangle> m_Triangles;

#ifdef ___CLING___
public:
    ClassDef(MDShapeTESSELLATED, 0)
#endif
};