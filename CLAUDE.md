# step_to_geomega — Codebase Overview

**Purpose:** Convert a STEP CAD assembly (as exported from Fusion 360 or similar)
into a MEGAlib / Geomega `.geo` geometry description — volume hierarchy,
placements, rotations, units, and naming — for use in gamma-ray detector
simulation (Cosima/Geant4 backend).

This file exists to orient a coding agent picking up this codebase: what each
file does, what's confirmed-correct vs. still guessed, and where the known
gaps are.

## Files

| File | Role |
|---|---|
| `step_to_geomega.cpp` | Main program. Reads STEP via OCCT's XCAF/XDE reader, walks the assembly tree, classifies each part's shape, and writes the `.geo` output. |
| `tessellatedsolid.h` / `.cpp` | Triangulates an arbitrary BRep shape into a vertex/triangle mesh and writes it as ASCII STL. Used for freeform parts, or for all parts with `--tessellate-all`. |
| `CMakeLists.txt` | Build file. Depends on OpenCascade via vcpkg (`find_package(OpenCASCADE CONFIG REQUIRED)`). |

## Build

```
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
```

Requires `opencascade` installed via vcpkg (`vcpkg install opencascade:x64-windows`).

## Usage

```
step_to_geomega.exe input.step output.geo [--tessellate-all]
```

Writes `output.geo` plus a companion `output_meshes/` folder containing `.stl`
files for freeform parts. Pass `--tessellate-all` to also mesh recognized
primitive parts such as boxes and cylinders.

## Architecture / data flow

1. **Read** — `STEPCAFControl_Reader` (not the plain `STEPControl_Reader`,
   which would flatten the assembly into one fused shape) reads the STEP
   file into an XCAF (`TDocStd_Document`) document, preserving assembly
   structure, instance names, and placements.
2. **Walk** — `BuildNode()` recursively walks the XCAF label tree via
   `XCAFDoc_ShapeTool`, building a `VolumeNode` tree. Each node holds its
   name, its shape (for leaf/part nodes), and its placement (`gp_Trsf`)
   *relative to its immediate parent* — this matches Geomega's own
   mother-relative placement model, so no extra coordinate math is needed
   at write time.
3. **Classify** — `ClassifyShape()` inspects each leaf part's BRep faces to
   detect:
   - **Axis-aligned box** (6 planar faces) → Geomega `BRIK`, using the exact
     bounding box (faithful, not an approximation, for a true box).
   - **Simple cylinder** (1 cylindrical face, ≤2 planar end caps) → Geomega
     `TUBE`. Heuristic: assumes the cylinder axis aligns with the bbox's
     longest dimension — wrong for tilted cylinders (falls through to the
     next case instead).
    - **Anything else (freeform/fillets/complex)** → falls back to a
       bounding-box `BRIK` **approximation**, flagged with a comment in the
       output, *and* additionally tessellated via `TessellatedSolid` into a
       companion `.stl` file for inspection/future use (see "Known gaps").
       Recognized primitives are tessellated only when `--tessellate-all` is
       passed.
4. **Write** — `WriteNode()` recursively emits Geomega `Volume` blocks:
   `.Material`, `.Shape`, `.Mother`, `.Position`, `.Rotation`. Names are
   sanitized (`SanitizeName()`) to strip spaces/punctuation and enforce
   uniqueness, since Geomega identifiers can't contain those characters.

## Known gaps / unverified assumptions

These were flagged during development because the Geomega manual's body
text (`doc/Geomega.pdf` in the MEGAlib repo) wasn't fully accessible at the
time — only its table of contents. **Confirm each of these against a local
MEGAlib checkout before trusting output on anything that matters:**

1. **Rotation matrix order.** `.Rotation` is emitted as a 9-value 3x3 matrix
   (chosen over 3 Euler angles specifically to avoid angle-order ambiguity),
   but the row-major element order is *unverified* against Geomega's actual
   parser. If rotated parts render tilted wrong, this is the first thing to
   check — likely fix is transposing the matrix.
2. **No confirmed tessellated/mesh shape keyword in Geomega.** Freeform
   parts get a real triangulated mesh written to `.stl` via
   `TessellatedSolid`, but there's currently no verified way to reference
   that mesh *as the actual shape* inside the `.geo` file — the `.geo`
   output still uses the bounding-box `BRIK` approximation as the
   syntax-safe fallback, with the `.stl` referenced only in a comment.
   `TessellatedSolid::WriteGeomegaFacetBlock()` contains an **explicitly
   experimental, unverified guess** at inline mesh syntax
   (`.Shape TESSELATED` / `.Shape.AddVertex` / `.Shape.AddTriangle`) — it is
   NOT called by default and should not be trusted without confirming
   against the real Geomega parser or source.
   - **Next step to close this gap:** grep a local MEGAlib install for the
     real mesh/shape class list, e.g.:
     `grep -ril "tessellat\|MDShapeSTL\|MDShapeTRIA" $MEGALIB/src/geomega/inc`
   - Also worth checking whether Cosima's Geant4 export layer accepts a
     GDML/STL insert as an alternative path in, if Geomega itself has no
     native mesh import.
3. **`.geo` vs. `.geo.setup` file extension.** MEGAlib's own shipped example
   geometries use the `.geo.setup` extension. This converter currently just
   writes whatever extension the user passes as `output.geo` — if Geomega
   doesn't recognize a plain `.geo` file, try renaming to `.geo.setup`.
4. **Materials are not translated.** STEP rarely carries usable material
   assignments (would require `XCAFDoc_MaterialTool` and a STEP file that
   actually populated it, which is uncommon from Fusion 360). Every volume
   is currently written with `.Material Vacuum` and a `TODO` comment —
   materials need to be set by hand per volume.
5. **Assembly (grouping) nodes get a placeholder shape.** Geomega requires
   every `Volume` to have a `Shape` line, so pure assembly/grouping nodes
   (no own solid, just children) get an oversized placeholder `BRIK` with
   `.Visibility 0`. This is a stand-in, not derived from the children's
   actual combined extent — resize by hand if it matters.
6. **Cylinder-axis heuristic** (see step 3 above) assumes the longest
   bounding-box dimension is the cylinder's axis. Tilted/short-and-fat
   cylinders may misclassify and fall through to the freeform path instead.

## Testing status

- **Validated by hand:** a single unrotated axis-aligned box (Fusion 360 →
  STEP → converter) produces the expected `BRIK` half-lengths and identity
  rotation matrix, with no `APPROXIMATION` flag — confirms the basic
  read/classify/write pipeline works.
- **Not yet validated:** rotated parts (rotation matrix order unconfirmed),
  cylinders, multi-level assemblies, freeform/tessellated parts loading
  successfully in actual Geomega, and overlap-checking via Geomega's
  built-in checker.

## Suggested next steps for whoever picks this up

1. Load a converted `.geo` file into actual Geomega (`geomega --help` for
   exact invocation — untested by whoever wrote this) and resolve any
   parse errors.
2. Confirm the `.Rotation` matrix order using a known-simple rotated test
   part; fix `WriteNode()`'s matrix emission if it's transposed.
3. Resolve the tessellated-shape question (see gap #2) and, once the real
   Geomega keyword is known, wire `TessellatedSolid`'s mesh data into the
   actual `.geo` `Shape` line for freeform parts instead of just the
   companion `.stl` file.
4. Consider adding material-mapping support (e.g. a user-supplied
   name→material lookup table) rather than defaulting everything to Vacuum.