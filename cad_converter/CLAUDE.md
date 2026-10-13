# CLAUDE.md

Guidance for Claude Code (claude.ai/code) when working in `cad_converter/`. See the repo-root `CLAUDE.md` for project-wide context (Linux release + Windows testing, vcpkg manifest-mode conventions), and [README.md](README.md) for build, usage and the pipeline.

## Requirements (from the user)

- Exactly one STEP file as input. The output is STL (default) or OBJ (`--obj`), with one named object per part (`--per-body`: per body), written next to the input or into `--output-dir`.
- Object names (default mode): the part's name if known, otherwise `mesh1`, `mesh2`, ... in output order. A part placed n > 1 times gets `_instanced<n>` appended; n counts placements, not bodies, because one placement can hold several bodies (`Body::partBodies`).
- Only watertight solids are converted. Shells and loose faces are removed when the STEP file is read, and solids still open after repair are removed with a warning. Every body is therefore a solid.
- Output feeds MEGAlib/Geant4. Every solid must be a closed 2-manifold (each edge on exactly two triangles, consistently oriented), and no two solids may overlap at all.
- Touching bodies should be flush. An overlap goes to the smaller body; the larger one loses that volume.
- Accuracy (max deviation from the true surface) matters.
- Mesh style: the simplest structured mesh on analytic solids (cylinder = full-height strips, cone = apex fan, sphere = seam-free lat/long grid; pole needles are accepted). Don't add interior points to cylinders or cones, and don't make the mesh isotropic.

## Build and test

`VCPKG_ROOT` must be set. Then: `cmake --preset windows`, `cmake --build --preset windows-release`, `ctest --preset windows-release` (Linux: `linux-release` / `linux-debug`).

- Each model in `tests/data` gets four tests: `convert.<model>`, `check.<model>`, `convert_stl.<model>` (into an `--output-dir`) and `stl_written.<model>`. `mesh_check` independently verifies closedness, orientation and the absence of overlaps on the `--per-body` OBJ. Unit tests: `naming`, `watertight`, and `shells_removed` (on `ShellTest.step`).
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
- `mesh_cut`: Manifold cuts of every overlapping solid pair, in rounds, with zero-thickness slits closed.
- `export`: grouping by part, and the STL/OBJ writers.

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
- **`ShapeAnalysis_Surface::NextValueOfUV` is slow on B-splines.** Its Newton step usually fails there and it falls back to a global search (milliseconds per point). `weld.cpp` uses `Extrema_GenLocateExtPS` from the same start point instead.
- **Manifold numbers meshes from a global counter, and output triangle order follows those IDs.** Building Manifolds in parallel would make the output order vary between runs. `mesh_cut.cpp` reserves the IDs up front (`ReserveIDs`) in a fixed order and passes them in as `runOriginalID`.
