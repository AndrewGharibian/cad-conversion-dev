// =============================================================================
// watertight_test — IsWatertight (step_import.cpp), which decides which
// solids are converted: a closed solid passes; a solid with an open shell, a
// shell and a face don't.
// =============================================================================
#include "step_import.hpp"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRep_Builder.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>

#include <iostream>

namespace
{
int failures = 0;

void Expect(bool got, bool expected, const char* what)
{
    if (got != expected)
    {
        ++failures;
        std::cout << "FAILED: " << what << ": IsWatertight gave " << (got ? "true" : "false") << "\n";
    }
}
} // namespace

int main()
{
    const TopoDS_Solid box = BRepPrimAPI_MakeBox(1.0, 2.0, 3.0).Solid();
    Expect(IsWatertight(box), true, "closed box");

    // The box without its last face: a solid whose shell is open.
    TopoDS_Shell open;
    BRep_Builder builder;
    builder.MakeShell(open);
    TopExp_Explorer face(box, TopAbs_FACE);
    for (int k = 0; k < 5; ++k, face.Next())
    {
        builder.Add(open, face.Current());
    }
    TopoDS_Solid leaky;
    builder.MakeSolid(leaky);
    builder.Add(leaky, open);
    Expect(IsWatertight(leaky), false, "solid with an open shell");

    Expect(IsWatertight(TopExp_Explorer(box, TopAbs_SHELL).Current()), false, "closed shell (not a solid)");
    Expect(IsWatertight(face.Current()), false, "face");

    TopoDS_Solid empty;
    builder.MakeSolid(empty);
    Expect(IsWatertight(empty), false, "solid without a shell");

    std::cout << (failures ? "FAILED" : "OK") << "\n";
    return failures ? 1 : 0;
}
