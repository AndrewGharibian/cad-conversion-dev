# Repository Guidelines

## Purpose and layout

This is a standalone C++17 converter from closed STL/OBJ meshes to MEGAlib /
Geomega geometry. It consumes existing meshes; it does not read CAD files or
generate meshes, and has no CAD-library dependencies.

- `src/meshfile.h` and `src/meshfile.cpp`: mesh data structures, ASCII/binary
  STL and OBJ parsing, polygon triangulation, vertex welding, and validation.
- `src/mesh_to_geomega.cpp`: CLI, naming, unit conversion, and output writers.
- `shapes/`: MEGAlib-dependent tessellated-shape prototype; excluded from the
  standalone CMake target.
- `examples/`: example Geomega geometry.
- `scripts/validate_box_mesh.ps1`: PowerShell validation of the example box's
  outward winding and signed volume; this is not a converter test suite.
- `inputs/`: local mesh fixtures. `output/`: generated geometry and mesh files.
- `README.md` and `CLAUDE.md`: usage and codebase overview; keep relevant
  behavior descriptions consistent when changing conversion behavior.

## Build and verification

Run commands from the repository root in WSL:

```sh
cmake -S . -B build
cmake --build build
```

There is no automated test target configured in CMake. For conversion changes,
build and exercise the relevant STL/OBJ paths. Write smoke-test output into a
temporary directory to avoid overwriting existing user output:

```sh
conversion_tmp=$(mktemp -d)
./build/mesh_to_geomega inputs/box2.stl "$conversion_tmp/box.geo.setup"
./build/mesh_to_geomega inputs/CollisionTest.obj "$conversion_tmp/collision.geo.setup"
```

Check output structure and diagnostics as appropriate. Parser or validation
changes should also exercise malformed inputs and verify nonzero exit codes.
The optional example-volume check requires PowerShell:
`pwsh -File scripts/validate_box_mesh.ps1`.

## Behavior to preserve

- Input coordinates are assumed to be millimeters and scaled by 0.1 to
  centimeters in Geomega output.
- Accept ASCII/binary STL and OBJ polygon faces with positive or negative
  indices. Validate finite coordinates, indices, degeneracy, closed manifold
  edges, consistent outward winding, and positive enclosed volume.
- Validate each OBJ `o` object independently. OBJ conversion produces the main
  setup file and a sibling `<output-name>_meshes/` directory containing each
  object's scaled, triangulated OBJ and included setup file. Preserve relative
  references so the files can be moved together.
- STL output embeds tessellated vertices and facets in its setup file.
- The documented local Geomega parser supports this shape format, but local
  Cosima Geant4 construction reports `TESSELLATED` volumes as unsupported.
  Converter success alone does not establish simulation compatibility.

## Editing conventions

Follow existing C++ style: four-space indentation, opening braces on separate
lines, and the current naming patterns. Keep the standalone converter free of
MEGAlib/CAD dependencies unless explicitly requested. Report actionable parse,
validation, and file I/O failures through the existing error handling pattern.

Respect existing local changes and data. Do not overwrite mesh fixtures,
generated output, or unrelated edits while implementing or validating changes.
Do not commit local build artifacts or incidental generated geometry.
