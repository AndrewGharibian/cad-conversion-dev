# cad_converter

Converts a STEP model into simulation-ready triangle meshes for MEGAlib /
Geant4:

**STEP → repair → make solids conform → mesh → resolve overlaps → STL/OBJ/Geomega**

Every solid in the output is a closed, consistently oriented surface (each
edge on exactly two triangles), no two solids overlap, and every triangle
stays within a known distance of the true CAD surface. Built on
[OpenCASCADE](https://dev.opencascade.org/) (OCCT 8.0.1) and
[Manifold](https://github.com/elalish/manifold).

## 1. Prerequisites (once per machine)

Dependencies come from [vcpkg](https://github.com/microsoft/vcpkg) in
**manifest mode**: [`vcpkg.json`](vcpkg.json) lists OpenCASCADE and Manifold,
and its `builtin-baseline` pins the exact vcpkg commit, so every machine gets
the same versions. Nothing from vcpkg is committed; it is restored during
CMake configure.

- **CMake >= 3.21** and a C++17 compiler (Windows: Visual Studio 2022 or
  newer with "Desktop development with C++"; Linux: GCC or Clang).
- **Linux only**: the system packages vcpkg needs to build OCCT, e.g. on
  Ubuntu/Debian:
  `sudo apt install build-essential git curl zip unzip tar pkg-config ninja-build libgl-dev libx11-dev`
- **vcpkg**, cloned somewhere outside this repository, with the `VCPKG_ROOT`
  environment variable pointing at it:

  ```bash
  git clone https://github.com/microsoft/vcpkg ~/vcpkg   # Windows: e.g. C:\dev\vcpkg
  ~/vcpkg/bootstrap-vcpkg.sh                             # Windows: bootstrap-vcpkg.bat
  export VCPKG_ROOT=~/vcpkg                              # Windows: setx VCPKG_ROOT C:\dev\vcpkg
  ```

  It must be a full git clone that contains the baseline commit; if configure
  complains about the baseline, run `git pull` inside it.

The first configure builds OCCT from source (tens of minutes); vcpkg caches
the result, so later configures restore it in seconds.

## 2. Build, test, install

All settings live in [`CMakePresets.json`](CMakePresets.json).

| | Windows | Linux |
|---|---|---|
| Configure | `cmake --preset windows` | `cmake --preset linux-release` |
| Build | `cmake --build --preset windows-release` | `cmake --build --preset linux-release` |
| Test | `ctest --preset windows-release` | `ctest --preset linux-release` |
| Executable | `build/windows/Release/cad_converter.exe` | `build/linux-release/cad_converter` |

`windows-debug` and `linux-debug` presets also exist.

For every model in [`tests/data`](tests/data), the tests:

- convert it to a per-body OBJ, failing on any `WARNING` or `ERROR`;
- check that OBJ independently with `mesh_check` ([`tests/mesh_check.cpp`](tests/mesh_check.cpp)):
  every solid is closed and consistently oriented, and no two solids overlap;
- convert it again with the default output (STL, one object per part) into
  an `--output-dir`, and check that the file is there;
- convert it to a Geomega geometry (`--geo`) and check it with `geo_check`
  ([`tests/geo_check.cpp`](tests/geo_check.cpp)) the way MEGAlib reads it:
  valid and unique names, the structure of every volume, and every mesh
  passing `MDShapeTessellated`'s own mesh check (ported into the test).

Unit tests check the object names (`naming`) and which bodies count as
watertight solids (`watertight`), and `shells_removed` checks that the open
sheet in `ShellTest.step` is removed.

Large models aren't committed. Put them in `tests/data/large/` (gitignored):
they get the `large` label, which the presets skip. Run them with
`ctest --test-dir build/windows -C Release -L large`.

To get a folder that can be copied to another machine:

```bash
cmake --install build/linux-release              # -> install/linux-release/{bin,lib}
cmake --install build/windows --config Release   # -> install/windows/bin (exe + DLLs)
```

## 3. Usage

```bash
cad_converter [options] model.step
```

It takes exactly one STEP file and writes `model.stl` next to it (or in
`--output-dir`). Each stage
prints its name and duration, followed by a mesh quality report. Problems are
printed as `WARNING` lines.

| Option | Effect |
|---|---|
| `--max-deviation <d>` | Accuracy of the mesh (largest distance from the true surface), in model units. Default: 1/1000 of the model's largest dimension. |
| `--obj` | Write `model.obj` (Wavefront OBJ, indexed vertices) instead of `model.stl`. |
| `--geo` | Write a MEGAlib/Geomega geometry instead of `model.stl`: `model.geo.setup` plus one `.geo` mesh file per object in `model_geo/` (see below). |
| `--output-dir <dir>` | Write the output file into `<dir>` (created if it doesn't exist) instead of next to the input. |
| `--per-body` | One object per body (`solid_0001`, `solid_0002`, ...) instead of one per part. |
| `--no-repair` | Skip checking and repairing the CAD geometry. |
| `--no-collisions` | Don't make the solids conform: overlaps stay and touching meshes aren't flush. |
| `--full-accuracy-check` | Measure every triangle's deviation (default: a sample of 200k on large models). |
| `--dump-fused <file.brep>` | Save the conformed model (no mesh), plus `<file.brep>.parts` and `<file.brep>.meshcut`. Passing that `.brep` as the input skips straight to meshing, for iterating on big models. |

**Output.** By default every **part** (STEP product) is one named object,
holding all of its placements, since they share a material. Each placement is
its own closed surface within the object. STL is multi-solid ASCII, with one
`solid <name> ... endsolid <name>` block per object. OBJ has one `o <name>`
object per object.

Objects are named as follows:

- **Known part name:** the object is named after it, for example
  `COS-DET-MEC-148_Detector_Indium`.
- **No name:** objects are numbered in output order: `mesh1`, `mesh2`, ...
- **Placed more than once:** the number of placements is appended, for
  example `92196A148_6-32x0.5_SHS_92196A148_instanced12`.
- **Spaces** become `_`, and a name used twice gets a `_2` suffix.

With `--per-body`, objects are named `solid_0001`, `solid_0002`, ... Coordinates are written with 17 significant digits, so vertices
shared by touching bodies stay bit-identical.

**Geomega output (`--geo`).** For MEGAlib, with the `TESSELLATED` shape
(`MDShapeTessellated`):

- `model_geo/<object>.geo`, one file per object: a volume per body
  (placement), with its `TESSELLATED` shape and embedded mesh
  (`<volume>_Shape.Vertex x y z` and `<volume>_Shape.Facet i j k`, 0-based,
  counter-clockwise seen from outside), placed in `World` at position 0.
  A part placed 12 times gives 12 volumes, `<object>_1` ... `<object>_12`:
  a Geomega volume has one shape, and MEGAlib needs a point inside each
  volume.
- `model.geo.setup`, the file MEGAlib loads: the material list, the
  surrounding sphere, the world volume, and for each object an `Include` of
  its mesh file followed by each volume's `Material`, `Color` and
  `Visibility`. **The materials are placeholders (`Aluminium`)**: set them
  there.

Lengths are in cm (the STEP model's mm divided by 10), each written with the
fewest digits that read back exactly. Geomega names allow only letters,
digits and `_`, so other characters become `_`, and names are made unique
ignoring case. Mesh file names are cut to 48 characters, since Windows paths
are limited to 260.

## 4. How it works

The pipeline is in [`src/main.cpp`](src/main.cpp); each stage has its own
source file, with the reasoning in its header comment.

1. **Read** ([`step_import`](src/step_import.cpp)): the STEP geometry and
   assembly tree, with every part's name. Only solids are kept: shells and
   loose faces are removed, because Geant4 and ROOT need every volume to be
   a watertight solid, and a surface encloses no volume. (In COSI these are
   the PCB copper layers, exported as zero-thickness faces.) A solid whose
   shell is still open after repair is removed too, with a `WARNING`.
2. **Repair** ([`repair`](src/repair.cpp), [`folded_faces`](src/folded_faces.cpp)):
   - each unique part is checked once and repaired if needed;
   - faces the import put on the wrong arc of a torus, sphere or cone are
     rebuilt;
   - every revolved face's seam is moved to U = 0, where OCCT's mesher
     expects it.
3. **Conform** ([`conform`](src/conform.cpp)): a general fuse makes touching
   solids share one face, so their meshes come out flush, and gives every
   overlap to the smaller solid.
   - Modeled screw threads in modeled tapped holes would become slivers too
     thin to mesh ([`thread_pairs`](src/thread_pairs.cpp)), so the smaller
     body of each such pair is left out of the fuse.
   - The fuse's result is checked. Where it breaks a pair, the smaller body
     is left out too, and only its neighbourhood is fused again.
4. **Mesh** ([`meshing`](src/meshing.cpp), [`revolved_mesh`](src/revolved_mesh.cpp),
   [`boundary_mesh`](src/boundary_mesh.cpp), [`weld`](src/weld.cpp)):
   - OCCT's mesher runs within the accuracy limit, with fallbacks for faces
     it can't handle;
   - full cylinders, cones and spheres get the simplest structured mesh;
   - each body is welded into one mesh and measured against the CAD surface.
5. **Resolve overlaps** ([`mesh_cut`](src/mesh_cut.cpp)):
   - every overlap left between two solid meshes is cut out of the larger
     body's mesh with Manifold's robust booleans. These come from bodies left
     out of the fuse, interferences the fuse missed, and triangles bulging
     past concave surfaces.
   - slivers (zero-area triangles, which ROOT and Geant4 reject) are
     removed by edge flips, without moving the surface.
   - every solid's mesh is checked to be closed. A defective fused solid
     sends its neighbourhood back to step 3.
6. **Export** ([`export`](src/export.cpp), [`geomega_export`](src/geomega_export.cpp)):
   one STL or OBJ file, or a Geomega geometry.
