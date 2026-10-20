# CLAUDE.md

Guidance for Claude Code (claude.ai/code) when working in `cad_converter/`. See the repo-root `CLAUDE.md` for project-wide context (Linux release + Windows testing, vcpkg manifest-mode conventions), and [README.md](README.md) for build, usage and the pipeline.

## Requirements (from the user)

- Exactly one STEP file as input. Outputs: OBJ (`--obj`), STL (`--stl`) and a MEGAlib/Geomega geometry (`--geo`), in any combination of the flags given; with none, OBJ only. One named object per part (`--per-body`: per body), written next to the input or into `--output-dir`.
- Geomega output (`--geo`): each object's mesh data goes in its own `.geo` file (one `TESSELLATED` volume per body, in the format `MDShapeTessellated` parses), included by `model.geo.setup`, where the materials are set (placeholders for now). The team's shape class and example files are in `../claude_resources/shapes/`, and the Geomega parser is in `../claude_resources/geomega/`.
- Object names (default mode): the part's name if known, otherwise `mesh1`, `mesh2`, ... in output order. A part placed n > 1 times gets `_instanced<n>` appended; n counts placements, not bodies, because one placement can hold several bodies (`Body::partBodies`).
- Only watertight solids are converted. Shells and loose faces are removed when the STEP file is read, and solids still open after repair are removed with a warning. Every body is therefore a solid.
- Output feeds MEGAlib/Geant4. Every solid must be a closed 2-manifold (each edge on exactly two triangles, consistently oriented), and no two solids may overlap at all.
- No degenerate facets: ROOT's `TGeoTessellated::FacetCheck` reports a facet under 1e-10 cm² (1e-8 mm²) as "degenerated" / "zero surface area". Slivers are flipped and tiny triangles (under 1e-7 mm²) collapsed, consistently in every body sharing the point, and after the cut only if no overlap results (re-cutting instead doesn't converge: it makes new tiny triangles).
- Touching bodies should be flush. An overlap goes to the smaller body; the larger one loses that volume.
- Accuracy (max deviation from the true surface) matters.
- Simplification options: `--angular-deflection <deg>` (default 20, `kDefaultMeshAngleDeg`; used by OCCT's mesher, the structured revolved faces and the boundary-mesh fallback) and `--min-volume <v>` (removes small solids right after reading and repair, before the fuse, so they take nothing from their neighbours).
- Mesh style: the simplest structured mesh on analytic solids (cylinder = full-height strips, cone = apex fan, sphere = seam-free lat/long grid; pole needles are accepted). Don't add interior points to cylinders or cones, and don't make the mesh isotropic.

## Build and test

`VCPKG_ROOT` must be set. Then: `cmake --preset windows`, `cmake --build --preset windows-release`, `ctest --preset windows-release` (Linux: `linux-release` / `linux-debug`).

- Each model in `tests/data` gets four tests: `convert.<model>`, `check.<model>`, `convert_stl.<model>` (into an `--output-dir`) and `stl_written.<model>`. `mesh_check` independently verifies closedness, orientation and the absence of overlaps on the `--per-body` OBJ. Each model also gets `convert_geo.<model>` and `geo_check.<model>`; `geo_check` reads the `--geo` output the way Geomega does and runs a port of `MDShapeTessellated::CheckMesh` on every volume. Other tests: `naming`, `watertight`, `shells_removed` (on `ShellTest.step`), `min_volume`, `angular_deflection` and the output-format tests `formats_default` / `formats_stl_geo`.
- Large models go in `tests/data/large/` (gitignored, label `large`).
- The build is warning-free (`/W3`, `-Wall -Wextra`). OCCT 8 deprecates the `TopTools_*` container names: use the aliases in `src/occt_types.hpp`.
- When changing the pipeline, compare outputs before and after. Outputs are deterministic, so unchanged behaviour means byte-identical STL/OBJ files.

## Code map (`src/`)

`main.cpp` runs the pipeline: read and prepare → conform → mesh and weld → cut and check, with local re-fuse passes → export. Each module's header comment explains its reasoning.

- `step_import`: XCAF reader with part names (keeps only solids), `IsWatertight`, `UniqueBodies` and `PlacedBodies`. Bodies keep `PlacedBodies` order through every stage.
- `repair`: `CheckAndRepair` (each unique part once) and `NormalizePeriodicSeams` (seam to U = 0).
- `folded_faces`: lunes (faces bounded by two meridians and axis points) that STEP import put on the wrong arc are rebuilt on the complementary arc.
- `thread_pairs`: freeform faces just inside another solid, i.e. modeled threads. The smaller body is left out of the fuse.
- `conform`: the general fuse (`FuseSolids`: the smallest owner keeps an overlap), the fuse check (`BrokenFuseResult`), local re-fuse (`RefuseLocally`, `RepairFuse`) and `PartnersOfMeshDefects`.
- `meshing`: OCCT meshing, with wire-restart and split fallbacks, and `boundary_mesh` as the last resort.
- `revolved_mesh`: the structured meshes.
- `weld`: per-body welding and deviation measurement.
- `mesh_cut`: Manifold cuts of every overlapping solid pair, in rounds, with zero-thickness slits closed; `FlipDegenerateTriangles` removes zero-area slivers, `CollapseTinyTriangles` tiny triangles.
- `export`: grouping by part, and the STL/OBJ writers.
- `geomega_export`: the `--geo` writer (names, units, file layout).

## OCCT 8.0.1 gotchas

- **`BRepTools_ReShape` matches by TShape *and* location by default.** To replace a part at every placement, set `ModeConsiderLocation() = true`.
- **`BRepTools_Modifier` on an assembly copies every placement.** Modify the unique parts (a compound of them), then put them back with ReShape.
- **Compare sub-shapes by TShape and location** (`ShapeIndexMap` / `IsSame`), not by TShape alone: a part can reuse one edge at several places.
- **Fuse history doesn't follow solids inside a compound argument.** Pass each solid piece separately (`FuseSolids` does).
- **The general fuse can silently miss interferences or break near-tangent contacts.** Hence the fuse check, the mesh check and the final all-pairs mesh cut.
- **Cone meshing depends on wire edge order.** A Fusion cone whose wire starts at the degenerate apex edge gets no triangulation; restarting the wire one edge later fixes it.
- **`ShapeUpgrade_ShapeDivideClosed` leaves stale SameParameter/SameRange flags.** Always follow it with `BRepLib::SameParameter(shape, tol, true)`.
- **`BRepMesh` overshoots its deflection by a few percent.** Ask for 0.9× the limit.
- **On Windows, `near` is a macro** (windef.h). Don't use it as a variable name.
- **OCCT's torus mesh can exceed the deflection by up to ~2x** when the deflection, not the angle, sets the spacing (a small `--max-deviation`, or a large `--angular-deflection`): `BRepMesh_TorusRangeSplitter` sizes each direction by its own sagitta, and the two add up. On cosi_test.step: 0.35 with `--max-deviation 0.2`, 1.71 vs 1.45 with `--angular-deflection 45`; within the limit at the defaults. Not fixed yet. A structured torus mesh (`revolved_mesh`) would fix it, but COSI's tori are half revolutions (SolidWorks splits them), and `revolved_mesh` only handles full ones (which is also why COSI gets the structured mesh on 0 faces). Lowering `DeflectionInterior` alone does nothing (face deflection = max of it and the edges'); a separate torus-first pass with `ForceFaceDeflection` broke the mesh.
- **`IMeshTools_Parameters::AngleInterior` defaults to twice `Angle`.** The torus and other range splitters use `Angle`, not `AngleInterior`.
- **`ShapeAnalysis_Surface::NextValueOfUV` is slow on B-splines.** Its Newton step usually fails there and it falls back to a global search (milliseconds per point). `weld.cpp` uses `Extrema_GenLocateExtPS` from the same start point instead.
- **Manifold numbers meshes from a global counter, and output triangle order follows those IDs.** Building Manifolds in parallel would make the output order vary between runs. `mesh_cut.cpp` reserves the IDs up front (`ReserveIDs`) in a fixed order and passes them in as `runOriginalID`.
