# v0.2 Voronoi-Informed Init + Crossing Repair — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add Voronoi-informed Imhof slot initialization and a deterministic post-solve crossing-repair pass to MakieTextRepel.jl, delivering guaranteed crossing-free label layouts under the existing public API.

**Architecture:** Single solver pass with structural inputs. Compute Voronoi cells of anchors via DelaunayTriangulation.jl, initialize each label at the highest-preference Imhof slot whose padded box fits inside the cell (TR fallback if none), run the existing `solve_repel` with those initial offsets, then a naive-O(n²) crossing scan + 2-opt position-swap repair pass with `max_iter = 100` backstop.

**Tech Stack:** Julia 1.11, Makie 0.24, GeometryBasics 0.5, DelaunayTriangulation.jl 1.6+.

**Specification:** `docs/superpowers/specs/2026-05-28-voronoi-init-and-crossing-repair-design.md`. Read the spec first; it owns the "why," this plan owns the "how."

---

## File structure

**Created in this plan:**
- `src/voronoi.jl` — `voronoi_cells`, `box_inside_polygon`
- `src/init.jl` — Imhof slot constants, `slot_offset`, `initial_offsets`
- `src/crossings.jl` — `Connector`, `connector_for`, `segments_cross`, `find_crossings`, `swap_positions!`, `repair_crossings!`
- `src/solvers/abstract.jl` — `AbstractClusterSolver`, `solve_cluster` interface
- `src/solvers/force.jl` — `ForceSolver` wrapping `solve_repel`
- `test/test_init.jl` — Voronoi cell + Imhof slot + initial-offsets unit tests
- `test/test_crossings.jl` — segments_cross, find_crossings, swap_positions!, repair_crossings! unit tests
- `CHANGELOG.md` — new, v0.2 first entry

**Modified:**
- `Project.toml` — add `DelaunayTriangulation` dep + compat; bump version to `0.2.0`
- `src/MakieTextRepel.jl` — add new includes
- `src/connectors.jl` — extract per-label connector geometry into `connector_for`; `build_connectors` calls through it
- `src/recipe.jl` — `lift` node calls Voronoi → init → solver → repair pipeline; pass `initial_offsets` via the existing `init_state` kwarg
- `test/runtests.jl` — include new test files
- `test/test_connectors.jl` — light edits for `connector_for` factoring
- `test/test_integration.jl` — add pipeline invariant test
- `CHANGELOG.md` — close out the "Unreleased" 0.1.0 section and add a new `## [0.2.0]` section above it

**Untouched by this plan:**
- `src/solver.jl` — `solve_repel` already accepts the `init_state` kwarg (from spike PR #10). `init_offsets` and `_GOLDEN_ANGLE` are RETAINED because `src/annotation_algorithm.jl` calls `init_offsets` for the `reset=true` warm-start at `src/annotation_algorithm.jl:176`. The new `src/init.jl` defines `initial_offsets` (with trailing `s`) so the names don't collide.
- `src/annotation_algorithm.jl` — not modified by this plan.
- `test/test_solver.jl` — not modified by this plan. Its existing `solve_repel` calls remain valid (they exercise the default `init_state === nothing` path and the kwarg signature is backwards-compatible).
- `test/test_annotation_algorithm.jl` — not modified.

---

## Type conventions used throughout

- Anchor positions: `Vector{Point2f}` (pixel-space).
- Label sizes (w, h): `Vector{Vec2f}`.
- Label offsets (delta from anchor to center): `Vector{Vec2f}`.
- Dropped flags: `BitVector`.
- Voronoi cells: `Vector{Union{Polygon, Nothing}}` where `Polygon` is `GeometryBasics.Polygon{2, Float32}` (a closed CCW polygon).
- RNG for DT.jl: `MersenneTwister(0)`.

The spec uses `Point2f` for offsets in a few places — that's a typo; v0.1 code is consistently `Vec2f` and this plan follows the code.

---

### Task 1: Add DelaunayTriangulation.jl dependency

**Files:**
- Modify: `Project.toml`

- [ ] **Step 1: Add the dependency declaration**

In `Project.toml`'s `[deps]` block (which currently lists `GeometryBasics`, `LinearAlgebra`, `Makie`, `TextMeasure` in that order), insert a `DelaunayTriangulation` line so the block stays alphabetized — between `GeometryBasics` and `LinearAlgebra`:

```toml
DelaunayTriangulation = "927a84f5-c5f4-47a5-9785-b46e178433df"
```

In the `[compat]` block (which currently lists `GeometryBasics`, `Makie`, `julia`), add — likewise alphabetized:

```toml
DelaunayTriangulation = "1.6"
```

- [ ] **Step 2: Regenerate the manifest and smoke-test the import**

Run:

```bash
cd /home/jonathanchen/projects/MakieTextRepel.jl/.claude/worktrees/v0.2-voronoi-crossings
julia --project=. -e 'using Pkg; Pkg.resolve(); Pkg.instantiate(); using DelaunayTriangulation; println(pkgversion(DelaunayTriangulation))'
```

Expected output: a version string like `1.6.6` (or newer).

- [ ] **Step 3: Confirm test suite still passes**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Expected: all existing tests pass. No regressions from adding the dep.

- [ ] **Step 4: Commit**

`Manifest.toml` is gitignored — only `Project.toml` changes are tracked.

```bash
git add Project.toml
git commit -m "Add DelaunayTriangulation.jl dependency for v0.2 Voronoi init"
```

---

### Task 2: `segments_cross` predicate

**Files:**
- Create: `src/crossings.jl`
- Create: `test/test_crossings.jl`
- Modify: `src/MakieTextRepel.jl`
- Modify: `test/runtests.jl`

- [ ] **Step 1: Write the failing tests**

Create `test/test_crossings.jl`:

```julia
using MakieTextRepel
using MakieTextRepel: segments_cross
using GeometryBasics
using Test

@testset "segments_cross" begin
    # Two segments forming an X
    @test segments_cross(Point2f(0, 0), Point2f(2, 2), Point2f(0, 2), Point2f(2, 0)) == true

    # Parallel non-coincident
    @test segments_cross(Point2f(0, 0), Point2f(2, 0), Point2f(0, 1), Point2f(2, 1)) == false

    # Disjoint, non-parallel
    @test segments_cross(Point2f(0, 0), Point2f(1, 0), Point2f(2, 1), Point2f(3, 2)) == false

    # Endpoint touch — not a crossing
    @test segments_cross(Point2f(0, 0), Point2f(1, 1), Point2f(1, 1), Point2f(2, 0)) == false

    # T-junction: one endpoint exactly on the other segment — not a crossing
    @test segments_cross(Point2f(0, 0), Point2f(2, 0), Point2f(1, 0), Point2f(1, 2)) == false

    # Collinear overlap — not a crossing
    @test segments_cross(Point2f(0, 0), Point2f(3, 0), Point2f(1, 0), Point2f(2, 0)) == false
end
```

- [ ] **Step 2: Wire the new test file into the suite**

Edit `test/runtests.jl` to add:

```julia
    include("test_crossings.jl")
```

inside the `@testset` after `include("test_integration.jl")`.

- [ ] **Step 3: Run the test, expect failure**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Expected: `UndefVarError: segments_cross not defined`.

- [ ] **Step 4: Implement `segments_cross`**

Create `src/crossings.jl`:

```julia
# crossings.jl — leader-segment crossing detection and 2-opt repair.

"""
Strict segment-segment intersection test via signed-area orientation.
Returns `true` only when the segments properly cross (each segment's
endpoints lie on opposite sides of the other line). Endpoint-touching
and collinear-overlap return `false`.
"""
function segments_cross(p1::Point2f, p2::Point2f, p3::Point2f, p4::Point2f)
    o(a, b, c) = (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1])
    d1 = o(p3, p4, p1)
    d2 = o(p3, p4, p2)
    d3 = o(p1, p2, p3)
    d4 = o(p1, p2, p4)
    return (d1 > 0 && d2 < 0 || d1 < 0 && d2 > 0) &&
           (d3 > 0 && d4 < 0 || d3 < 0 && d4 > 0)
end
```

- [ ] **Step 5: Add the include to the module**

Edit `src/MakieTextRepel.jl`, replacing the existing include list with:

```julia
include("geometry.jl")
include("solver.jl")
include("connectors.jl")
include("crossings.jl")
include("measure.jl")
include("recipe.jl")
```

- [ ] **Step 6: Run, expect pass**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Expected: all tests pass, including the new `segments_cross` testset.

- [ ] **Step 7: Commit**

```bash
git add src/crossings.jl src/MakieTextRepel.jl test/test_crossings.jl test/runtests.jl
git commit -m "Add segments_cross strict-intersection predicate"
```

---

### Task 3: `box_inside_polygon` predicate

**Files:**
- Create: `src/voronoi.jl`
- Create: `test/test_init.jl`
- Modify: `src/MakieTextRepel.jl`
- Modify: `test/runtests.jl`

- [ ] **Step 1: Write the failing test**

Create `test/test_init.jl`:

```julia
using MakieTextRepel
using MakieTextRepel: box_inside_polygon
using GeometryBasics
using Test

@testset "box_inside_polygon" begin
    # CCW square (10×10) at origin
    poly = Polygon([Point2f(0, 0), Point2f(10, 0), Point2f(10, 10), Point2f(0, 10)])

    # Box entirely inside
    @test box_inside_polygon(Rect2f(2, 2, 4, 4), poly) == true

    # Box with one corner just outside
    @test box_inside_polygon(Rect2f(8, 8, 4, 4), poly) == false

    # Box entirely outside
    @test box_inside_polygon(Rect2f(20, 20, 4, 4), poly) == false

    # Triangular cell
    tri = Polygon([Point2f(0, 0), Point2f(10, 0), Point2f(0, 10)])
    @test box_inside_polygon(Rect2f(1, 1, 2, 2), tri) == true
    @test box_inside_polygon(Rect2f(5, 5, 2, 2), tri) == false  # crosses hypotenuse
end
```

- [ ] **Step 2: Wire into the suite**

Edit `test/runtests.jl` to add `include("test_init.jl")` inside the testset.

- [ ] **Step 3: Run, expect failure**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Expected: `UndefVarError: box_inside_polygon not defined`.

- [ ] **Step 4: Implement `box_inside_polygon`**

Create `src/voronoi.jl`:

```julia
# voronoi.jl — Voronoi cell computation + cell-fit predicate.

using DelaunayTriangulation
using Random
const DT = DelaunayTriangulation

"""
Test whether `box`'s four corners all lie inside the convex polygon `poly`.
Uses sign-of-cross-product against each edge; consistent edge winding (CCW)
required. Sufficient for boxes inside convex Voronoi cells (which remain
convex after clipping with the convex viewport rectangle).
"""
function box_inside_polygon(box::Rect2f, poly::GeometryBasics.Polygon)
    pts = decompose(Point2f, poly.exterior)
    n = length(pts)
    n < 3 && return false
    corners = (Point2f(box.origin),
               Point2f(box.origin[1] + box.widths[1], box.origin[2]),
               Point2f(box.origin .+ box.widths),
               Point2f(box.origin[1], box.origin[2] + box.widths[2]))
    for c in corners
        for k in 1:n
            a = pts[k]
            b = pts[k % n + 1]
            # Cross of edge a→b with point a→c. CCW polygon => interior has cross > 0.
            cr = (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1])
            cr < 0 && return false
        end
    end
    return true
end
```

- [ ] **Step 5: Update the module includes**

Edit `src/MakieTextRepel.jl`:

```julia
include("geometry.jl")
include("voronoi.jl")
include("solver.jl")
include("connectors.jl")
include("crossings.jl")
include("measure.jl")
include("recipe.jl")
```

- [ ] **Step 6: Run, expect pass**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

- [ ] **Step 7: Commit**

```bash
git add src/voronoi.jl src/MakieTextRepel.jl test/test_init.jl test/runtests.jl
git commit -m "Add box_inside_polygon convex point-in-polygon predicate"
```

---

### Task 4: `voronoi_cells` function

**Files:**
- Modify: `src/voronoi.jl`
- Modify: `test/test_init.jl`

- [ ] **Step 1: Add the failing tests**

Append to `test/test_init.jl`:

```julia
using MakieTextRepel: voronoi_cells

@testset "voronoi_cells" begin
    viewport = Rect2f(0, 0, 100, 100)

    # n = 0 → empty
    @test voronoi_cells(Point2f[], viewport) == Union{GeometryBasics.Polygon, Nothing}[]

    # n = 1 → single nothing slot (need ≥ 3 for triangulation)
    @test voronoi_cells([Point2f(50, 50)], viewport) == [nothing]

    # n = 2 → two nothing slots
    @test voronoi_cells([Point2f(25, 50), Point2f(75, 50)], viewport) == [nothing, nothing]

    # n = 3 non-collinear → three real cells
    anchors = [Point2f(25, 25), Point2f(75, 25), Point2f(50, 75)]
    cells = voronoi_cells(anchors, viewport)
    @test length(cells) == 3
    @test all(c !== nothing for c in cells)

    # Defensive: returned polygons are CCW-wound (positive signed shoelace area).
    # If DT.jl ever changes its winding convention, box_inside_polygon silently
    # inverts; this assertion is the canary.
    for c in cells
        c === nothing && continue
        pts = decompose(Point2f, c.exterior)
        m = length(pts)
        area = 0.0
        for k in 1:m
            a = pts[k]; b = pts[k % m + 1]
            area += a[1] * b[2] - b[1] * a[2]
        end
        @test area > 0
    end

    # Coincident anchors → both nothing
    anchors = [Point2f(25, 25), Point2f(25, 25), Point2f(75, 75), Point2f(60, 60)]
    cells = voronoi_cells(anchors, viewport)
    @test cells[1] === nothing
    @test cells[2] === nothing
    @test cells[3] !== nothing
    @test cells[4] !== nothing

    # NaN anchor → nothing in that slot, others fine
    anchors = [Point2f(25, 25), Point2f(75, 25), Point2f(50, 75), Point2f(NaN, NaN)]
    cells = voronoi_cells(anchors, viewport)
    @test cells[4] === nothing
    @test cells[1] !== nothing

    # Determinism: same input → same cells across two calls
    a1 = voronoi_cells([Point2f(10, 10), Point2f(50, 50), Point2f(90, 90), Point2f(10, 90)], viewport)
    a2 = voronoi_cells([Point2f(10, 10), Point2f(50, 50), Point2f(90, 90), Point2f(10, 90)], viewport)
    @test a1 == a2
end
```

- [ ] **Step 2: Run, expect failure**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Expected: `UndefVarError: voronoi_cells not defined`.

- [ ] **Step 3: Implement `voronoi_cells`**

Append to `src/voronoi.jl`:

```julia
"""
Compute Voronoi cells for `anchors` clipped to `viewport`. Returns a vector
of length `length(anchors)` where each entry is either a `GeometryBasics.Polygon`
(the clipped cell for that anchor, with CCW exterior) or `nothing` for anchors
that are non-finite, coincident with another anchor, or part of an input with
fewer than three distinct finite anchor coordinates.

Determinism: distinct coordinates are sorted lexicographically before triangulation;
DT.jl's RNG is explicitly seeded with `MersenneTwister(0)`; cells are mapped back
to all anchors via the (x, y) → cell dictionary.
"""
function voronoi_cells(anchors::Vector{Point2f}, viewport::Rect2f)
    n = length(anchors)
    cells = Vector{Union{GeometryBasics.Polygon, Nothing}}(nothing, n)

    # 1. Identify finite anchors.
    finite = falses(n)
    for i in 1:n
        a = anchors[i]
        finite[i] = isfinite(a[1]) && isfinite(a[2])
    end

    # 2. Count coordinate occurrences (finite anchors only).
    counts = Dict{Tuple{Float32, Float32}, Int}()
    for i in 1:n
        finite[i] || continue
        k = (anchors[i][1], anchors[i][2])
        counts[k] = get(counts, k, 0) + 1
    end

    # 3. Collect distinct finite (x, y) values, sorted lex.
    distinct = sort!(collect(keys(counts)))
    length(distinct) < 3 && return cells   # all nothing

    # 4. Triangulate distinct points; clip Voronoi cells to viewport.
    rng = MersenneTwister(0)
    points = [(Float64(p[1]), Float64(p[2])) for p in distinct]
    tri = DT.triangulate(points; rng = rng)
    vor = DT.voronoi(tri; clip = true, clip_polygon = _viewport_clip(viewport))

    # 5. Build coord → cell mapping.
    # DT.jl returns CCW-wound rings, closed (first == last). Drop the closing duplicate.
    coord_to_cell = Dict{Tuple{Float32, Float32}, GeometryBasics.Polygon}()
    for (idx, coord) in enumerate(distinct)
        poly_pts = DT.get_polygon_coordinates(vor, idx)
        ring = [Point2f(Float32(pt[1]), Float32(pt[2])) for pt in poly_pts[1:end-1]]
        coord_to_cell[coord] = GeometryBasics.Polygon(ring)
    end

    # 6. Assign cells to non-coincident finite anchors only.
    # An anchor with `counts[k] > 1` is coincident with at least one other label —
    # leave its cell as `nothing` so both labels fall through to TR Imhof fallback.
    for i in 1:n
        finite[i] || continue
        k = (anchors[i][1], anchors[i][2])
        counts[k] == 1 && (cells[i] = coord_to_cell[k])
    end

    return cells
end

"""DT.jl's `clip_polygon` requires `(points, boundary_nodes)` CCW Tuple form."""
function _viewport_clip(viewport::Rect2f)
    o = viewport.origin
    w = viewport.widths
    pts = [(Float64(o[1]),       Float64(o[2])),
           (Float64(o[1] + w[1]), Float64(o[2])),
           (Float64(o[1] + w[1]), Float64(o[2] + w[2])),
           (Float64(o[1]),        Float64(o[2] + w[2]))]
    return (pts, [1, 2, 3, 4, 1])
end
```

Note: The test "Coincident anchors → both nothing" has 4 anchors with `(25,25)` doubled, plus `(75,75)` and `(60,60)`. That's 3 distinct finite coordinates → triangulation proceeds → cells exist for `(75,75)` and `(60,60)`, while both `(25,25)` slots get `nothing` because their `counts` entry is 2. The mapping in step 6 only assigns a cell when `counts[k] == 1`.

- [ ] **Step 4: Run, expect pass**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

If a DT.jl API mismatch surfaces (e.g., `get_polygon_coordinates` returns a different shape than expected), check the current DT.jl docs at https://juliageometry.github.io/DelaunayTriangulation.jl/stable/api/voronoi/ and adapt the polygon-extraction step.

- [ ] **Step 5: Commit**

```bash
git add src/voronoi.jl test/test_init.jl
git commit -m "Add voronoi_cells with DT.jl backing, finite/coincident filters, viewport clipping"
```

---

### Task 5: Imhof slot constants and `slot_offset` helper

**Files:**
- Create: `src/init.jl`
- Modify: `src/MakieTextRepel.jl`
- Modify: `test/test_init.jl`

- [ ] **Step 1: Add the failing tests**

Append to `test/test_init.jl`:

```julia
using MakieTextRepel: slot_offset, IMHOF_ORDER

@testset "Imhof slots" begin
    p = 2.0f0
    w = 10.0f0
    h = 6.0f0

    # All 8 directions, with anchor at origin and label center at the returned offset.
    # Verify each slot positions the label so the anchor sits outside the axis-aligned
    # padded box (one edge of the box is at distance p from the anchor).
    expected = Dict(
        :TR => Vec2f(p + w/2,  p + h/2),
        :R  => Vec2f(p + w/2,  0),
        :T  => Vec2f(0,        p + h/2),
        :BR => Vec2f(p + w/2, -p - h/2),
        :L  => Vec2f(-p - w/2, 0),
        :BL => Vec2f(-p - w/2, -p - h/2),
        :B  => Vec2f(0,       -p - h/2),
        :TL => Vec2f(-p - w/2,  p + h/2),
    )
    for (slot, expect) in expected
        @test slot_offset(slot, Vec2f(w, h), p) ≈ expect
    end

    # Preference order
    @test IMHOF_ORDER == (:TR, :R, :T, :BR, :L, :BL, :B, :TL)
end
```

- [ ] **Step 2: Run, expect failure**

Expected: `UndefVarError: slot_offset not defined`.

- [ ] **Step 3: Implement `init.jl`**

Create `src/init.jl`:

```julia
# init.jl — Imhof-preferred slot selection for label initialization.

"""Imhof preference order (TR most preferred; TL least). See Imhof 1962, Liao et al. 2024."""
const IMHOF_ORDER = (:TR, :R, :T, :BR, :L, :BL, :B, :TL)

"""
Offset (label center − anchor) for one of the 8 Imhof slots, positioned so the
anchor lies just outside the axis-aligned padded box of the label. `size` is
the unpadded label size (w, h); `p` is `point_padding`.
"""
function slot_offset(slot::Symbol, size::Vec2f, p::Real)
    p32 = Float32(p)
    w = size[1]
    h = size[2]
    hw = w / 2
    hh = h / 2
    slot === :TR && return Vec2f( p32 + hw,  p32 + hh)
    slot === :R  && return Vec2f( p32 + hw,  0f0)
    slot === :T  && return Vec2f( 0f0,       p32 + hh)
    slot === :BR && return Vec2f( p32 + hw, -p32 - hh)
    slot === :L  && return Vec2f(-p32 - hw,  0f0)
    slot === :BL && return Vec2f(-p32 - hw, -p32 - hh)
    slot === :B  && return Vec2f( 0f0,      -p32 - hh)
    slot === :TL && return Vec2f(-p32 - hw,  p32 + hh)
    error("unknown slot: $slot")
end
```

- [ ] **Step 4: Update the module includes**

Edit `src/MakieTextRepel.jl`:

```julia
include("geometry.jl")
include("voronoi.jl")
include("init.jl")
include("solver.jl")
include("connectors.jl")
include("crossings.jl")
include("measure.jl")
include("recipe.jl")
```

- [ ] **Step 5: Run, expect pass**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

- [ ] **Step 6: Commit**

```bash
git add src/init.jl src/MakieTextRepel.jl test/test_init.jl
git commit -m "Add Imhof slot constants and slot_offset helper"
```

---

### Task 6: `initial_offsets` function

**Files:**
- Modify: `src/init.jl`
- Modify: `test/test_init.jl`

- [ ] **Step 1: Add failing tests**

Append to `test/test_init.jl`:

```julia
using MakieTextRepel: initial_offsets, RepelParams

@testset "initial_offsets" begin
    # Sparse: each anchor's cell easily fits a small label at TR. Expect every offset
    # to equal the TR slot.
    viewport = Rect2f(0, 0, 200, 200)
    anchors = [Point2f(50, 50), Point2f(150, 50), Point2f(100, 150)]
    sizes = [Vec2f(10, 6), Vec2f(10, 6), Vec2f(10, 6)]
    params = RepelParams(point_padding = 2.0, box_padding = 0.0)
    cells = voronoi_cells(anchors, viewport)

    offsets = initial_offsets(anchors, sizes, cells, params)
    @test length(offsets) == 3
    expected_tr = slot_offset(:TR, sizes[1], params.point_padding)
    @test offsets[1] ≈ expected_tr
    @test offsets[2] ≈ expected_tr
    @test offsets[3] ≈ expected_tr

    # n = 1: cell is nothing → TR fallback.
    cells1 = voronoi_cells([Point2f(50, 50)], viewport)
    offs1 = initial_offsets([Point2f(50, 50)], [Vec2f(10, 6)], cells1, params)
    @test offs1[1] ≈ expected_tr

    # only_move = :x → y-component zeroed in initial offset
    params_x = RepelParams(point_padding = 2.0, only_move = :x)
    offs_x = initial_offsets(anchors, sizes, cells, params_x)
    @test all(o -> o[2] == 0f0, offs_x)

    # only_move = :y → x-component zeroed
    params_y = RepelParams(point_padding = 2.0, only_move = :y)
    offs_y = initial_offsets(anchors, sizes, cells, params_y)
    @test all(o -> o[1] == 0f0, offs_y)

    # Determinism
    @test initial_offsets(anchors, sizes, cells, params) == initial_offsets(anchors, sizes, cells, params)
end
```

- [ ] **Step 2: Run, expect failure**

Expected: `UndefVarError: initial_offsets not defined`.

- [ ] **Step 3: Implement `initial_offsets`**

Append to `src/init.jl`:

```julia
"""
Initial offsets for each anchor: pick the highest-preference Imhof slot whose
padded box fits inside the anchor's Voronoi cell; fall back to TR if none fit
or the cell is `nothing`. Apply `_constrain(offset, params.only_move)` to
respect axis-lock semantics from the first iteration.

Pure function of (anchors, sizes, cells, params). Same inputs → same outputs.
"""
function initial_offsets(anchors::Vector{Point2f}, sizes::Vector{Vec2f},
                         cells::Vector{<:Union{GeometryBasics.Polygon, Nothing}},
                         params::RepelParams)
    n = length(anchors)
    offsets = Vector{Vec2f}(undef, n)
    pad = Float32(params.box_padding)
    p = params.point_padding
    for i in 1:n
        cell = cells[i]
        chosen = :TR
        if cell !== nothing
            for slot in IMHOF_ORDER
                candidate = slot_offset(slot, sizes[i], p)
                padded_size = sizes[i] .+ 2pad
                box = box_at(anchors[i], candidate, padded_size)
                if box_inside_polygon(box, cell)
                    chosen = slot
                    break
                end
            end
        end
        raw_off = slot_offset(chosen, sizes[i], p)
        offsets[i] = _constrain(raw_off, params.only_move)
    end
    return offsets
end
```

- [ ] **Step 4: Run, expect pass**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

- [ ] **Step 5: Commit**

```bash
git add src/init.jl test/test_init.jl
git commit -m "Add initial_offsets: Voronoi-informed Imhof slot selection with only_move respected"
```

---

### Task 7: Factor `Connector` + `connector_for` out of `connectors.jl`

**Files:**
- Modify: `src/connectors.jl`
- Modify: `test/test_connectors.jl` (light addition)
- Modify: `test/test_crossings.jl` (add direct connector_for tests)

- [ ] **Step 1: Add tests for the new helper to `test/test_crossings.jl`**

Append to `test/test_crossings.jl`:

```julia
using MakieTextRepel: connector_for, Connector

@testset "connector_for" begin
    params = RepelParams(box_padding = 4.0, point_padding = 2.0)
    # Label offset to the right of anchor with non-zero length leader → drawn.
    c = connector_for(Point2f(0, 0), Vec2f(20, 0), Vec2f(10, 6), false, params, 2.0)
    @test c.drawn == true

    # Dropped label → not drawn.
    c2 = connector_for(Point2f(0, 0), Vec2f(20, 0), Vec2f(10, 6), true, params, 2.0)
    @test c2.drawn == false

    # Anchor inside padded box (offset 0) → not drawn.
    c3 = connector_for(Point2f(0, 0), Vec2f(0, 0), Vec2f(10, 6), false, params, 2.0)
    @test c3.drawn == false

    # Visible length below min_segment_length → not drawn.
    c4 = connector_for(Point2f(0, 0), Vec2f(11, 0), Vec2f(10, 6), false, params, 100.0)
    @test c4.drawn == false
end
```

- [ ] **Step 2: Run, expect failure**

Expected: `UndefVarError: connector_for not defined`.

- [ ] **Step 3: Refactor `connectors.jl`**

Replace `src/connectors.jl` entirely:

```julia
# connectors.jl — per-label connector geometry + flat segment construction.

"""
Resolved connector geometry for one label. `drawn = false` mirrors the v0.1
suppression rules in `build_connectors`: dropped label, anchor inside padded
box, trim direction inversion, or visible length below `min_segment_length`.
"""
struct Connector
    label_end::Point2f
    anchor_end::Point2f
    drawn::Bool
end

const _UNDRAWN = Connector(Point2f(0, 0), Point2f(0, 0), false)

"""
Compute the connector geometry for a single (anchor, offset, size, dropped)
tuple. Single source of truth for `build_connectors` and `repair_crossings!` —
the `drawn` flag is the predicate both use to decide whether to render or scan
the segment.
"""
function connector_for(anchor::Point2f, offset::Vec2f, size::Vec2f,
                       dropped::Bool, params::RepelParams, min_len::Real)
    dropped && return _UNDRAWN
    pad = Float32(params.box_padding)
    ppad = Float32(params.point_padding)
    min_len_f = Float32(min_len)
    psize = size .+ 2pad
    box = box_at(anchor, offset, psize)
    edge = clip_to_box_edge(box, anchor)
    edge === nothing && return _UNDRAWN
    dir = edge .- anchor
    dlen = norm(dir)
    dlen <= ppad && return _UNDRAWN
    seg_start = anchor .+ (ppad / dlen) .* dir
    norm(edge .- seg_start) <= min_len_f && return _UNDRAWN
    return Connector(edge, Point2f(seg_start), true)
end

"""
Build flat `Point2f` endpoint pairs for `linesegments!` (pixel space).
Delegates per-label decisions to `connector_for`.
"""
function build_connectors(anchors::Vector{Point2f}, offsets::Vector{Vec2f},
                          sizes::Vector{Vec2f}, dropped::BitVector,
                          min_len::Real, box_padding::Real;
                          point_padding::Real = 0.0)
    # Build a transient params for connector_for. Only the padding fields are read.
    params = RepelParams(box_padding = box_padding, point_padding = point_padding)
    segs = Point2f[]
    for i in eachindex(anchors)
        c = connector_for(anchors[i], offsets[i], sizes[i], dropped[i], params, min_len)
        c.drawn && push!(segs, c.anchor_end, c.label_end)
    end
    return segs
end
```

- [ ] **Step 4: Run all tests**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Expected: all pass — the existing `test_connectors.jl` should still pass because `build_connectors` produces identical output via the new code path.

- [ ] **Step 5: Commit**

```bash
git add src/connectors.jl test/test_crossings.jl
git commit -m "Factor Connector + connector_for: shared geometry for build_connectors and crossing repair"
```

---

### Task 8: `find_crossings` function

**Files:**
- Modify: `src/crossings.jl`
- Modify: `test/test_crossings.jl`

- [ ] **Step 1: Add failing tests**

Append to `test/test_crossings.jl`:

```julia
using MakieTextRepel: find_crossings

@testset "find_crossings" begin
    # Two connectors that cross.
    c1 = Connector(Point2f(2, 0), Point2f(0, 2), true)
    c2 = Connector(Point2f(2, 2), Point2f(0, 0), true)
    @test find_crossings([c1, c2]) == [(1, 2)]

    # Same but second one undrawn → no crossing.
    c2_off = Connector(c2.label_end, c2.anchor_end, false)
    @test find_crossings([c1, c2_off]) == Tuple{Int,Int}[]

    # Three connectors, two pairwise crossings.
    c3 = Connector(Point2f(0, 0), Point2f(2, 2), true)
    c4 = Connector(Point2f(2, 0), Point2f(0, 2), true)
    c5 = Connector(Point2f(0, 1), Point2f(2, 1), true)
    crossings = find_crossings([c3, c4, c5])
    @test (1, 2) in crossings
    @test issorted(crossings)  # lex-ordered

    # No crossings on parallel lines.
    c6 = Connector(Point2f(0, 0), Point2f(2, 0), true)
    c7 = Connector(Point2f(0, 1), Point2f(2, 1), true)
    @test find_crossings([c6, c7]) == Tuple{Int,Int}[]
end
```

- [ ] **Step 2: Run, expect failure**

Expected: `UndefVarError: find_crossings not defined`.

- [ ] **Step 3: Implement `find_crossings`**

Append to `src/crossings.jl`:

```julia
"""
Pairwise O(n²) scan over `connectors`. Returns lex-ordered `(i, j)` index
pairs with `i < j` for every pair whose segments strictly cross.
Undrawn connectors (`drawn == false`) are skipped.
"""
function find_crossings(connectors::Vector{Connector})
    crossings = Tuple{Int,Int}[]
    n = length(connectors)
    for i in 1:n
        connectors[i].drawn || continue
        for j in (i+1):n
            connectors[j].drawn || continue
            if segments_cross(connectors[i].anchor_end, connectors[i].label_end,
                              connectors[j].anchor_end, connectors[j].label_end)
                push!(crossings, (i, j))
            end
        end
    end
    return crossings
end
```

- [ ] **Step 4: Run, expect pass**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

- [ ] **Step 5: Commit**

```bash
git add src/crossings.jl test/test_crossings.jl
git commit -m "Add find_crossings: naive O(n²) scan returning lex-ordered crossing pairs"
```

---

### Task 9: `swap_positions!` function

**Files:**
- Modify: `src/crossings.jl`
- Modify: `test/test_crossings.jl`

- [ ] **Step 1: Add failing test**

Append to `test/test_crossings.jl`:

```julia
using MakieTextRepel: swap_positions!

@testset "swap_positions!" begin
    anchors = [Point2f(0, 0), Point2f(10, 0)]
    offsets = [Vec2f(5, 5), Vec2f(-5, 5)]
    # absolute positions: (5, 5) and (5, 5)
    # Yes those are coincident — pick more interesting ones:
    offsets = [Vec2f(2, 3), Vec2f(-1, 4)]
    # absolute positions: (2, 3) and (9, 4)
    swap_positions!(offsets, anchors, 1, 2)
    # After: label 1 at (9, 4), label 2 at (2, 3)
    # offsets[1] = (9, 4) - (0, 0) = (9, 4)
    # offsets[2] = (2, 3) - (10, 0) = (-8, 3)
    @test offsets[1] ≈ Vec2f(9, 4)
    @test offsets[2] ≈ Vec2f(-8, 3)
end
```

- [ ] **Step 2: Run, expect failure**

Expected: `UndefVarError: swap_positions! not defined`.

- [ ] **Step 3: Implement `swap_positions!`**

Append to `src/crossings.jl`:

```julia
"""
Exchange absolute positions of labels `i` and `j` while preserving their
(label_text → anchor) identity. The new offsets are computed so each label's
absolute position becomes the other's old absolute position.
"""
function swap_positions!(offsets::Vector{Vec2f}, anchors::Vector{Point2f}, i::Int, j::Int)
    pos_i_old = anchors[i] .+ offsets[i]
    pos_j_old = anchors[j] .+ offsets[j]
    offsets[i] = Vec2f(pos_j_old .- anchors[i])
    offsets[j] = Vec2f(pos_i_old .- anchors[j])
    return offsets
end
```

- [ ] **Step 4: Run, expect pass**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

- [ ] **Step 5: Commit**

```bash
git add src/crossings.jl test/test_crossings.jl
git commit -m "Add swap_positions!: exchange absolute positions preserving anchor identity"
```

---

### Task 10: `repair_crossings!` function

**Files:**
- Modify: `src/crossings.jl`
- Modify: `test/test_crossings.jl`

- [ ] **Step 1: Add failing tests**

Append to `test/test_crossings.jl`:

```julia
using MakieTextRepel: repair_crossings!

@testset "repair_crossings!" begin
    # Construct a 2-label crossing: anchors on x-axis, labels swapped across.
    anchors = [Point2f(0, 0), Point2f(10, 0)]
    offsets = [Vec2f(12, 4), Vec2f(-12, 4)]
    # absolute positions: label_1 at (12, 4) — leader (0,0)→(12,4); label_2 at (-2, 4) — leader (10,0)→(-2,4). Cross.
    sizes = [Vec2f(4, 2), Vec2f(4, 2)]
    dropped = BitVector([false, false])
    params = RepelParams(box_padding = 1.0, point_padding = 0.5)

    iters = repair_crossings!(offsets, anchors, sizes, dropped, params; min_len = 0.5)
    @test iters ≤ 5  # should converge in 1 iteration
    # Verify no crossings remain after repair.
    connectors = [connector_for(anchors[i], offsets[i], sizes[i], dropped[i], params, 0.5)
                  for i in eachindex(anchors)]
    @test isempty(find_crossings(connectors))

    # Sum of center-distance has decreased.
    @test norm(offsets[1]) + norm(offsets[2]) ≤ norm(Vec2f(12, 4)) + norm(Vec2f(-12, 4))

    # Dropped label: its slot is skipped, others repair around it.
    anchors2 = [Point2f(0, 0), Point2f(10, 0), Point2f(5, 5)]
    offsets2 = [Vec2f(12, 4), Vec2f(-12, 4), Vec2f(0, 0)]
    sizes2 = [Vec2f(4, 2), Vec2f(4, 2), Vec2f(4, 2)]
    dropped2 = BitVector([false, false, true])
    repair_crossings!(offsets2, anchors2, sizes2, dropped2, params; min_len = 0.5)
    # Label 3 should never appear in any crossing pair → its offset is unchanged.
    @test offsets2[3] == Vec2f(0, 0)

    # max_iter cap: pathological alternating input doesn't hang.
    # (Hard to construct a true cycle in floating point, so just assert termination.)
    offsets3 = [Vec2f(12, 4), Vec2f(-12, 4)]
    iters3 = repair_crossings!(offsets3, anchors, sizes, dropped, params; max_iter = 3, min_len = 0.5)
    @test iters3 ≤ 3
end
```

- [ ] **Step 2: Run, expect failure**

Expected: `UndefVarError: repair_crossings! not defined`.

- [ ] **Step 3: Implement `repair_crossings!`**

Append to `src/crossings.jl`:

```julia
"""
Iterate: scan for crossings; for each non-conflicting crossing pair swap label
positions; repeat. Terminates when no crossings remain or `max_iter` exceeded.
`min_len` matches `min_segment_length` from the recipe so the scan agrees with
what `build_connectors` would render.

Returns the number of outer iterations consumed (0 if no crossings on first scan).
"""
function repair_crossings!(offsets::Vector{Vec2f}, anchors::Vector{Point2f},
                           sizes::Vector{Vec2f}, dropped::BitVector,
                           params::RepelParams; min_len::Real = 2.0, max_iter::Int = 100)
    for iter in 1:max_iter
        connectors = [connector_for(anchors[i], offsets[i], sizes[i], dropped[i], params, min_len)
                      for i in eachindex(offsets)]
        crossings = find_crossings(connectors)
        isempty(crossings) && return iter - 1

        swapped = Set{Int}()
        for (i, j) in crossings
            (i in swapped || j in swapped) && continue
            swap_positions!(offsets, anchors, i, j)
            push!(swapped, i)
            push!(swapped, j)
        end
    end
    return max_iter
end
```

- [ ] **Step 4: Run, expect pass**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

- [ ] **Step 5: Commit**

```bash
git add src/crossings.jl test/test_crossings.jl
git commit -m "Add repair_crossings!: scan + non-conflicting batched swaps with max_iter backstop"
```

---

### Task 11: Pin the spike-landed `init_state` kwarg + `RepelParams` copy constructor

**Files:**
- Modify: `test/test_solver.jl`

**Intent:** Spike PR #10 already landed (a) `solve_repel(...; init_state, obstacles, pin_mask, pinned_offsets)` returning a `(; offsets, dropped, iter, residual)` NamedTuple (see `src/solver.jl:94-198`), and (b) `RepelParams(base::RepelParams; kwargs...)` copy constructor (see `src/solver.jl:25-29`). Tasks 12 and 13 depend on both. Lock the contract with a focused test so any future refactor that drops these stays caught at the unit level.

`src/solver.jl` is **not modified by this task**. `init_offsets` and `_GOLDEN_ANGLE` are retained throughout — `src/annotation_algorithm.jl:176` calls `init_offsets` for the `reset=true` warm-start path.

- [ ] **Step 1: Add the pinning testset**

Append to `test/test_solver.jl`:

```julia
@testset "v0.2 prereqs: init_state kwarg and RepelParams copy constructor" begin
    # `init_state` kwarg path — fresh start overridden, output deterministic.
    anchors = [Point2f(0, 0), Point2f(40, 0)]
    sizes   = [Vec2f(10, 4), Vec2f(10, 4)]
    init    = [Vec2f(10, 0), Vec2f(-10, 0)]
    p = RepelParams(box_padding = 0.0, max_iter = 200)

    r = solve_repel(anchors, sizes, p; init_state = init)
    @test r isa NamedTuple
    @test Set(propertynames(r)) == Set([:offsets, :dropped, :iter, :residual])
    @test length(r.offsets) == 2
    @test all(isfinite, r.offsets)

    # Wrong-length init_state must throw.
    @test_throws DimensionMismatch solve_repel(anchors, sizes, p;
                                               init_state = [Vec2f(0, 0)])

    # Copy constructor — only listed field changes; others carried over verbatim.
    base = RepelParams(force = (3.0, 3.0), max_iter = 123, box_padding = 7.5)
    bumped = RepelParams(base; max_iter = 999)
    @test bumped.max_iter == 999
    @test bumped.force == (3.0, 3.0)
    @test bumped.box_padding == 7.5

    # Bounds override leaves everything else intact.
    withb = RepelParams(base; bounds = Rect2f(0, 0, 100, 100))
    @test withb.bounds == Rect2f(0, 0, 100, 100)
    @test withb.max_iter == 123
end
```

- [ ] **Step 2: Run, expect pass**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Expected: all tests pass. The new testset validates that the spike's API surface is what Tasks 12 and 13 need.

- [ ] **Step 3: Commit**

```bash
git add test/test_solver.jl
git commit -m "Pin v0.2 prereqs: solve_repel init_state kwarg + RepelParams copy ctor"
```

---

### Task 12: `AbstractClusterSolver` + `ForceSolver`

**Files:**
- Create: `src/solvers/abstract.jl`
- Create: `src/solvers/force.jl`
- Modify: `src/MakieTextRepel.jl`
- Modify: `test/test_solver.jl`

- [ ] **Step 1: Add failing test**

Append to `test/test_solver.jl`:

```julia
using MakieTextRepel: AbstractClusterSolver, ForceSolver, solve_cluster

@testset "ForceSolver wraps solve_repel" begin
    anchors = [Point2f(0, 0), Point2f(20, 0)]
    sizes = [Vec2f(6, 4), Vec2f(6, 4)]
    init = [Vec2f(5, 5), Vec2f(-5, 5)]
    bounds = Rect2f(-50, -50, 100, 100)
    params = RepelParams(bounds = bounds)
    solver = ForceSolver(params)

    o1, d1 = solve_cluster(solver, anchors, sizes, init, bounds)
    r      = solve_repel(anchors, sizes, params; init_state = init)
    @test o1 == r.offsets
    @test d1 == r.dropped
end
```

- [ ] **Step 2: Run, expect failure**

Expected: `UndefVarError: ForceSolver not defined`.

- [ ] **Step 3: Create `src/solvers/abstract.jl`**

```julia
# solvers/abstract.jl — Internal cluster-solver interface (v0.3 candidate for export).

"""
Marker type for cluster fallback solvers. Concrete subtypes implement
`solve_cluster(s, anchors, sizes, initial_offsets, bounds)` returning
`(offsets::Vector{Vec2f}, dropped::BitVector)`. Internal in v0.2; will be
exposed publicly when a second implementation lands (see GitHub issue #8).
"""
abstract type AbstractClusterSolver end

function solve_cluster end
```

- [ ] **Step 4: Create `src/solvers/force.jl`**

```julia
# solvers/force.jl — Force-directed cluster solver wrapping solve_repel.

"""ForceSolver carries `RepelParams` and dispatches to `solve_repel`."""
struct ForceSolver <: AbstractClusterSolver
    params::RepelParams
end

function solve_cluster(s::ForceSolver, anchors::Vector{Point2f}, sizes::Vector{Vec2f},
                       initial_offsets::Vector{Vec2f}, bounds::Rect2f)
    # `RepelParams(base; ...)` (src/solver.jl:25-29) copies `s.params` and overrides
    # only `bounds`, so callers don't need to mutate anything.
    p = RepelParams(s.params; bounds = bounds)
    r = solve_repel(anchors, sizes, p; init_state = initial_offsets)
    return (r.offsets, r.dropped)
end
```

- [ ] **Step 5: Update module includes**

Edit `src/MakieTextRepel.jl` (current state, post-spike):

```julia
include("geometry.jl")
include("solver.jl")
include("connectors.jl")
include("measure.jl")
include("recipe.jl")
include("annotation_algorithm.jl")
```

Insert the v0.2 includes so `voronoi.jl`/`init.jl` come after `solver.jl` (they don't depend on it but stay grouped), and `crossings.jl` comes after `connectors.jl` (it depends on `connector_for` from Task 7). The `solvers/` files go after `solver.jl`. `annotation_algorithm.jl` stays last because it depends on `solver.jl` and `recipe.jl` is unchanged. Final order:

```julia
include("geometry.jl")
include("solver.jl")
include("solvers/abstract.jl")
include("solvers/force.jl")
include("voronoi.jl")
include("init.jl")
include("connectors.jl")
include("crossings.jl")
include("measure.jl")
include("recipe.jl")
include("annotation_algorithm.jl")
```

- [ ] **Step 6: Run, expect pass**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

- [ ] **Step 7: Commit**

```bash
git add src/solvers/ src/MakieTextRepel.jl test/test_solver.jl
git commit -m "Add AbstractClusterSolver + ForceSolver wrapping solve_repel"
```

---

### Task 13: Wire the new pipeline in `recipe.jl`

**Files:**
- Modify: `src/recipe.jl`

- [ ] **Step 1: Modify the `lift` node in `Makie.plot!`**

Replace the body of the `solved = lift(...) do ... end` at lines 74–90 of `src/recipe.jl` with:

```julia
    solved = lift(p.px_anchors, p.text, p.fontsize, p.font,
                  p.force, p.force_point, p.force_pull, p.max_iter, p.only_move,
                  p.box_padding, p.point_padding, p.max_overlaps, bounds_obs, p.min_segment_length) do px, labels, fs, font,
                                                                                                       fr, frp, fpl, mi, om,
                                                                                                       bp, pp, mo, bnds, ml
        anchors = [Point2f(q[1], q[2]) for q in px]
        sizes = measure_labels(labels, font, fs, 1.0)
        params = RepelParams(; force = Tuple(Float64.(fr)),
                               force_point = Tuple(Float64.(frp)),
                               force_pull = Tuple(Float64.(fpl)),
                               max_iter = Int(mi), only_move = Symbol(om),
                               box_padding = Float64(bp), point_padding = Float64(pp),
                               max_overlaps = Float64(mo), bounds = bnds)
        clip = bnds === nothing ? Rect2f(-1f6, -1f6, 2f6, 2f6) : bnds
        cells = voronoi_cells(anchors, clip)
        init = initial_offsets(anchors, sizes, cells, params)
        offsets, dropped = solve_cluster(ForceSolver(params), anchors, sizes, init, clip)
        repair_crossings!(offsets, anchors, sizes, dropped, params; min_len = Float64(ml))
        (; anchors, sizes, offsets, dropped, params)
    end
```

Notes:
- `solve_cluster` (Task 12) internally calls `solve_repel(...; init_state = init)` and destructures the NamedTuple to return the `(offsets, dropped)` tuple shape that the lift body consumes.
- `clip` is the fallback for when `bnds === nothing`, which is defensive — the recipe path always sets bounds via `bounds_obs` (`src/recipe.jl:69-71`).
- `min_segment_length` joins the lift's input list because `repair_crossings!` uses it to decide which connectors are visible (and therefore eligible to cross).

- [ ] **Step 2: Expose lifted state on the plot attributes**

The existing `add_input!` call at line 93 only exposes `:computed_offsets`. Add four more so the integration test (Task 14) can read the rest. Replace line 93 with:

```julia
    Makie.add_input!(p.attributes, :computed_offsets, lift(s -> s.offsets, solved))
    Makie.add_input!(p.attributes, :computed_anchors, lift(s -> s.anchors, solved))
    Makie.add_input!(p.attributes, :computed_sizes,   lift(s -> s.sizes,   solved))
    Makie.add_input!(p.attributes, :computed_dropped, lift(s -> s.dropped, solved))
    Makie.add_input!(p.attributes, :computed_params,  lift(s -> s.params,  solved))
```

Two notes:
- `bounds_obs` from line 69–71 is always a `Rect2f` (clamped to viewport size), so the `bnds === nothing` fallback is defensive — `bnds` should never be `nothing` for the recipe path.
- `min_segment_length` is now a `lift` input because `repair_crossings!` uses it.

- [ ] **Step 3: Rebaseline `test/test_integration.jl`**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Existing integration tests in `test/test_integration.jl` may assert specific offset values that change because (a) Imhof initialization replaces the golden-angle initial positions for the recipe path, and (b) `repair_crossings!` can swap labels post-solve. For any failing numerical comparison, grep the failure output for the actual computed value and update the test. Tests asserting behavioral properties (`@test all(isfinite, ...)`, `@test boxes don't overlap`, etc.) should still pass — if not, that's a regression to investigate, not rebaseline.

`test/test_solver.jl` and `test/test_annotation_algorithm.jl` should **not** need rebaselining: they call `solve_repel` directly with the default `init_state === nothing` branch, which still routes through the retained `init_offsets`/`_GOLDEN_ANGLE` path.

**Do not delete `init_offsets` or `_GOLDEN_ANGLE`.** `src/annotation_algorithm.jl:176` calls `init_offsets(anchors, psizes, alg.params)` for its `reset=true` warm-start, and the default-init branch of `solve_repel` (`src/solver.jl:109`) calls it when `init_state === nothing`. Removing them would break the spike-PR API surface.

- [ ] **Step 4: Smoke-test the example**

```bash
julia --project=. examples/readme_example.jl
```

The example should run without errors and produce a PNG that visually does not have crossing leader lines. Manual eyeball check.

- [ ] **Step 5: Commit**

```bash
git add src/recipe.jl test/test_integration.jl
git commit -m "Wire Voronoi init + repair_crossings! pipeline in recipe"
```

---

### Task 14: Integration invariant test

**Files:**
- Modify: `test/test_integration.jl`

- [ ] **Step 1: Add the test**

Append to `test/test_integration.jl`. The file already imports `CairoMakie` and `MakieTextRepel`, so `Figure`/`Axis`/`textrepel!` are in scope; only the two crossing helpers need explicit import:

```julia
using MakieTextRepel: connector_for, find_crossings

within_bounds(pos::Point2f, vp::Rect2f) =
    vp.origin[1] <= pos[1] <= vp.origin[1] + vp.widths[1] &&
    vp.origin[2] <= pos[2] <= vp.origin[2] + vp.widths[2]

@testset "v0.2 pipeline invariants" begin
    # Three case fixtures spanning sparsity regimes.
    cases = [
        (name = "sparse",
         anchors = Point2f[(0.2, 0.2), (0.8, 0.2), (0.5, 0.8), (0.2, 0.8), (0.8, 0.8)],
         labels = ["a", "b", "c", "d", "e"],
         limits = (0, 1, 0, 1)),
        (name = "dense",
         anchors = Point2f[(0.5, 0.5), (0.51, 0.51), (0.49, 0.49), (0.5, 0.48)],
         labels = ["alpha", "beta", "gamma", "delta"],
         limits = (0.4, 0.6, 0.4, 0.6)),
        (name = "mixed",
         anchors = Point2f[(0.1, 0.1), (0.9, 0.9), (0.5, 0.51), (0.52, 0.49), (0.48, 0.5)],
         labels = ["isolated_a", "isolated_b", "clu1", "clu2", "clu3"],
         limits = (0, 1, 0, 1)),
    ]

    for case in cases
        @testset "$(case.name)" begin
            fig = Figure(size = (400, 400))
            ax = Axis(fig[1, 1], limits = case.limits)
            plt = textrepel!(ax, case.anchors; text = case.labels)
            Makie.update_state_before_display!(fig)

            # Reach into the lifted state.
            solved = plt.attributes[:computed_offsets][]
            # We need anchors / sizes / dropped / params too — expose them via plt.attributes
            # in Task 13's recipe edit if not already (see plan's note on Observable names).

            # Build connectors and assert no crossings.
            # NOTE: the test references the same fields the recipe's solved NamedTuple carries.
            # If the recipe stores these on plt.attributes via add_input!, read them; otherwise
            # rerun the pipeline on the same inputs for the test.

            @test all(isfinite, solved)
        end
    end
end
```

Task 13 step 2 already exposed `:computed_offsets`, `:computed_anchors`, `:computed_sizes`, `:computed_dropped`, `:computed_params` on `plt.attributes`. This test reads them.

Replace the placeholder `@test all(isfinite, solved)` above with the full assertion block:

```julia
            offsets = plt.attributes[:computed_offsets][]
            anchors = plt.attributes[:computed_anchors][]
            sizes   = plt.attributes[:computed_sizes][]
            dropped = plt.attributes[:computed_dropped][]
            params  = plt.attributes[:computed_params][]
            min_len = plt.min_segment_length[]

            connectors = [connector_for(anchors[i], offsets[i], sizes[i], dropped[i], params, min_len)
                          for i in eachindex(offsets)]
            # Pixel-space viewport in axis-scene-local coordinates — same frame as `anchors`.
            vp = params.bounds
            @test all(isfinite, offsets)
            @test all(i -> dropped[i] || within_bounds(anchors[i] + offsets[i], vp), eachindex(offsets))
            @test isempty(find_crossings(connectors))
```

- [ ] **Step 2: Run, expect pass**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

- [ ] **Step 3: Add a `max_overlaps` pipeline interaction test**

Append to `test/test_integration.jl` after the invariant testset above:

```julia
@testset "v0.2 max_overlaps interaction" begin
    # Crowded layout: 6 labels packed into a small area. max_overlaps = 2 should
    # drop the most crowded ones. We assert: (a) at least one label dropped,
    # (b) connectors for non-dropped labels still have no crossings.
    anchors = Point2f[(0.50, 0.50), (0.51, 0.50), (0.50, 0.51),
                      (0.49, 0.50), (0.50, 0.49), (0.51, 0.51)]
    labels = ["aa", "bb", "cc", "dd", "ee", "ff"]

    fig = Figure(size = (400, 400))
    ax = Axis(fig[1, 1], limits = (0.4, 0.6, 0.4, 0.6))
    plt = textrepel!(ax, anchors; text = labels, max_overlaps = 2)
    Makie.update_state_before_display!(fig)

    offsets   = plt.attributes[:computed_offsets][]
    anchors_p = plt.attributes[:computed_anchors][]
    sizes_p   = plt.attributes[:computed_sizes][]
    dropped   = plt.attributes[:computed_dropped][]
    params    = plt.attributes[:computed_params][]
    min_len   = plt.min_segment_length[]

    @test count(dropped) ≥ 1   # crowded enough that at least one was dropped
    connectors = [connector_for(anchors_p[i], offsets[i], sizes_p[i], dropped[i], params, min_len)
                  for i in eachindex(offsets)]
    @test isempty(find_crossings(connectors))   # repair pass doesn't corrupt visible labels
end
```

- [ ] **Step 4: Commit**

```bash
git add test/test_integration.jl
git commit -m "Add v0.2 pipeline invariant tests: no crossings on sparse, dense, mixed inputs"
```

---

### Task 15: Project.toml version bump + CHANGELOG entry

**Files:**
- Modify: `Project.toml`
- Modify: `CHANGELOG.md` (already exists, created by spike PR #10 in Keep a Changelog format)

- [ ] **Step 1: Bump version**

Edit `Project.toml` line 4:

```toml
version = "0.2.0"
```

- [ ] **Step 2: Update CHANGELOG.md**

The file opens with an `## [Unreleased]` heading whose body documents the 0.1.0 feature set (it predates the v0.2 work). Rename that heading to `## [0.1.0] - YYYY-MM-DD` (use today's date; this closes out the 0.1.0 section), then add a new `## [0.2.0] - YYYY-MM-DD` section directly above it:

```markdown
## [0.2.0] - YYYY-MM-DD

Layouts are now initialized using Imhof-preferred slots within each anchor's Voronoi cell when geometry allows, and a post-solve repair pass guarantees no crossing leader lines. Existing user code runs unchanged; output positions will differ from v0.1.

### Added

- Voronoi-informed initialization (`src/voronoi.jl`, `src/init.jl`) — labels start at the highest-preference Imhof slot (TR > R > T > BR > L > BL > B > TL) that fits inside their anchor's Voronoi cell, with TR fallback when none fit.
- Crossing repair (`src/crossings.jl`) — deterministic 2-opt position-swap pass guarantees no leader-line crossings in the rendered output.
- Internal `AbstractClusterSolver` interface (`src/solvers/`) — seam for future Julia-native solver implementations.
- New dependency: `DelaunayTriangulation.jl` (≥ 1.6).

### Changed

- `textrepel!` recipe pipeline now Voronoi-initializes label positions and runs a post-solve crossing-repair pass. Output offsets differ from v0.1 even for identical inputs.

### Notes

- `solve_repel`'s `init_state` kwarg (added in 0.1.0 via the annotation-algorithm spike) is now the channel through which the recipe injects Imhof-derived initial positions.
- `init_offsets` and `_GOLDEN_ANGLE` (`src/solver.jl`) are retained: they back the default branch of `solve_repel` and are called by `TextRepelAlgorithm` for warm-starts.
```

- [ ] **Step 3: Commit**

```bash
git add Project.toml CHANGELOG.md
git commit -m "Bump to 0.2.0 and document Voronoi init + crossing repair in CHANGELOG"
```

---

### Task 16: Final end-to-end sanity

**Files:** none (verification only)

- [ ] **Step 1: Full test suite**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

All tests pass.

- [ ] **Step 2: Run the example**

```bash
julia --project=. examples/readme_example.jl
```

Open the produced PNG and confirm:
1. No leader lines cross.
2. Labels are placed at sensible (Imhof-preferred) positions when geometry allows.
3. The output looks at least as good as v0.1.

- [ ] **Step 3: Optional: regenerate the README hero image**

The current `assets/example.png` was produced with v0.1. If the v0.2 output is materially different (likely just better), regenerate:

```bash
julia --project=. examples/readme_example.jl
mv example.png assets/example.png  # or wherever the example writes
git add assets/example.png
git commit -m "Regenerate README hero image with v0.2 layouts"
```

- [ ] **Step 4: Manual review of the diff**

```bash
git log --oneline main..HEAD
git diff --stat main
```

Confirm the diff scope matches the spec's "Module layout" section: 5 new source files (`voronoi.jl`, `init.jl`, `crossings.jl`, `solvers/abstract.jl`, `solvers/force.jl`), 2 new test files (`test_init.jl`, `test_crossings.jl`), modifications to `Project.toml`/`CHANGELOG.md`/`src/MakieTextRepel.jl`/`src/connectors.jl`/`src/recipe.jl`/`test/runtests.jl`/`test/test_solver.jl` (Task 11 pinning testset only)/`test/test_connectors.jl`/`test/test_integration.jl`.

`src/solver.jl` and `src/annotation_algorithm.jl` should be **unchanged**. If `git diff main -- src/solver.jl src/annotation_algorithm.jl` shows anything, investigate — that's a regression.

No extra files touched. No scope creep.

---

## Notes for the implementer

- **Read the spec first.** This plan tells you HOW; the spec tells you WHY. `docs/superpowers/specs/2026-05-28-voronoi-init-and-crossing-repair-design.md`.
- **Commit frequently.** Each task commit message is in the plan; don't combine tasks into a single commit.
- **DT.jl API drift.** If the polygon-extraction step in Task 4 doesn't produce the expected shape, check the current DT.jl docs. The library's API changed somewhat between 0.x and 1.x; we target 1.6+.
- **Float32 discipline.** v0.1 is consistent about `Float32` for pixel-space math (`Vec2f`, `Point2f`). Keep this in new code — accidental `Float64` allocations are a perf regression even when the numbers are equivalent.
- **Determinism is the contract.** If you find yourself reaching for `rand()` or unsorted `Dict` iteration, stop. There's a deterministic alternative.
- **The `connector_for` factoring (Task 7) is a refactor**, not a feature add. Existing connector tests should pass unchanged.

## Reference: GitHub issues

- #8 — Future Julia-native solver implementations (ODE, AD-energy, symbolic). The `AbstractClusterSolver` interface added in Task 12 is the seam.
- #9 — Future Bentley-Ottmann replacement for `find_crossings` at high label counts.
