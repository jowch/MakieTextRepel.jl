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
- `src/solver.jl` — `solve_repel` takes required `initial_offsets::Vector{Vec2f}`; `init_offsets` and `_GOLDEN_ANGLE` deleted
- `src/connectors.jl` — extract per-label connector geometry into `connector_for`; `build_connectors` calls through it
- `src/recipe.jl` — `lift` node calls Voronoi → init → solver → repair pipeline
- `test/runtests.jl` — include new test files
- `test/test_solver.jl` — pass explicit initial offsets; rebaseline determinism expected values
- `test/test_connectors.jl` — light edits for `connector_for` factoring
- `test/test_integration.jl` — add pipeline invariant test

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

Edit `Project.toml` `[deps]` block (after line 7) to add:

```toml
DelaunayTriangulation = "927a84f5-c5f4-47a5-9785-b46e178433df"
```

Then in the `[compat]` block (after line 19) add:

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

### Task 11: Add `initial_offsets` argument to `solve_repel` (no behavior change)

**Files:**
- Modify: `src/solver.jl`
- Modify: `test/test_solver.jl`
- Modify: `src/recipe.jl`

**Intent:** Change the signature only; keep `init_offsets` and `_GOLDEN_ANGLE` for now and have callers pass `init_offsets(anchors, psizes, params)` explicitly. Output is byte-identical to v0.1. The deletion of `init_offsets`/`_GOLDEN_ANGLE` happens in Task 13 once the new pipeline replaces them at the call sites.

- [ ] **Step 1: Update `src/solver.jl`**

Replace the `solve_repel` signature (line 82) with:

```julia
function solve_repel(anchors::Vector{Point2f}, sizes::Vector{Vec2f},
                     initial_offsets::Vector{Vec2f}, p::RepelParams)
```

And replace line 88:

```julia
    offsets = [_constrain(o, p.only_move) for o in initial_offsets]
```

(was: `offsets = [_constrain(o, p.only_move) for o in init_offsets(anchors, psizes, p)]`)

The rest of the function is unchanged. Do **not** delete `_GOLDEN_ANGLE` or `init_offsets` in this task.

- [ ] **Step 2: Find all `solve_repel` call sites**

```bash
grep -rn 'solve_repel(' src/ test/
```

Expected hits: `src/recipe.jl:87`, `test/test_solver.jl` (~15 lines). Each call needs an explicit `initial_offsets` argument.

- [ ] **Step 3: Update `src/recipe.jl` line 87**

Before:
```julia
        offsets, dropped = solve_repel(anchors, sizes, params)
```

After:
```julia
        psizes = [s .+ 2 * Float32(params.box_padding) for s in sizes]
        init = init_offsets(anchors, psizes, params)
        offsets, dropped = solve_repel(anchors, sizes, init, params)
```

- [ ] **Step 4: Update every `solve_repel` call site in `test/test_solver.jl`**

For each call, compute `psizes = [s .+ 2*Float32(params.box_padding) for s in sizes]` then pass `init_offsets(anchors, psizes, params)`. For example:

Before:
```julia
offsets, dropped = solve_repel(anchors, sizes, params)
```

After:
```julia
psizes = [s .+ 2*Float32(params.box_padding) for s in sizes]
init = init_offsets(anchors, psizes, params)
offsets, dropped = solve_repel(anchors, sizes, init, params)
```

Since `init_offsets` is unchanged in this task and still produces the same golden-angle offsets, every existing test assertion remains valid — no rebaselining needed.

- [ ] **Step 5: Run, expect pass**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Expected: all tests pass unchanged. No numerical assertions broke.

- [ ] **Step 6: Commit**

```bash
git add src/solver.jl src/recipe.jl test/test_solver.jl
git commit -m "Add required initial_offsets argument to solve_repel (no behavior change)"
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
    o2, d2 = solve_repel(anchors, sizes, init, params)
    @test o1 == o2
    @test d1 == d2
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
# solvers/force.jl — Force-directed cluster solver wrapping the v0.1 solve_repel.

"""ForceSolver carries `RepelParams` and dispatches to `solve_repel`."""
struct ForceSolver <: AbstractClusterSolver
    params::RepelParams
end

function solve_cluster(s::ForceSolver, anchors::Vector{Point2f}, sizes::Vector{Vec2f},
                       initial_offsets::Vector{Vec2f}, bounds::Rect2f)
    # Apply bounds via a params shallow-copy so callers don't need to mutate.
    p = RepelParams(
        force = s.params.force, force_point = s.params.force_point, force_pull = s.params.force_pull,
        max_iter = s.params.max_iter, only_move = s.params.only_move,
        box_padding = s.params.box_padding, point_padding = s.params.point_padding,
        max_overlaps = s.params.max_overlaps, step_max = s.params.step_max,
        pull_threshold = s.params.pull_threshold, tol = s.params.tol,
        bounds = bounds,
    )
    return solve_repel(anchors, sizes, initial_offsets, p)
end
```

- [ ] **Step 5: Update module includes**

Edit `src/MakieTextRepel.jl`:

```julia
include("geometry.jl")
include("voronoi.jl")
include("init.jl")
include("solver.jl")
include("solvers/abstract.jl")
include("solvers/force.jl")
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
git add src/solvers/ src/MakieTextRepel.jl test/test_solver.jl
git commit -m "Add AbstractClusterSolver + ForceSolver wrapping solve_repel"
```

---

### Task 13: Wire the new pipeline in `recipe.jl`

**Files:**
- Modify: `src/recipe.jl`

- [ ] **Step 1: Modify the `lift` node in `Makie.plot!`**

Replace the body of the `solved = lift(...) do ... end` at lines 74–89 of `src/recipe.jl` with:

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
        cells = voronoi_cells(anchors, bnds === nothing ? Rect2f(-1f6, -1f6, 2f6, 2f6) : bnds)
        init = initial_offsets(anchors, sizes, cells, params)
        offsets, dropped = solve_cluster(ForceSolver(params), anchors, sizes, init,
                                          bnds === nothing ? Rect2f(-1f6, -1f6, 2f6, 2f6) : bnds)
        repair_crossings!(offsets, anchors, sizes, dropped, params; min_len = Float64(ml))
        (; anchors, sizes, offsets, dropped, params)
    end
```

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

- [ ] **Step 3: Run, expect pass**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Existing integration tests (test_integration.jl) need rebaselining at this point. If any assert specific offset values, comment them out, get the new values, write them back.

- [ ] **Step 4: Delete the now-unused `init_offsets` and `_GOLDEN_ANGLE`**

The new pipeline (`initial_offsets` from `src/init.jl`) replaces these at the only remaining call site (`recipe.jl` from Step 1 above). Delete them.

In `src/solver.jl`, delete lines 19 and 21–47 (the `_GOLDEN_ANGLE` constant declaration and the `init_offsets` function and its docstring).

In `test/test_solver.jl`:
- Update line 1's `using` to drop `init_offsets`: `using MakieTextRepel: RepelParams, box_at`
- Delete the entire `@testset "init_offsets" begin ... end` block (lines 5 through ~40 — the testset that calls `init_offsets` directly).

Also in `test/test_solver.jl`, every `solve_repel` call that was updated in Task 11 to use `init_offsets(anchors, psizes, params)` for its explicit `init` argument now needs a different source for `init`. Replace each `init_offsets(...)` call with `zeros(Vec2f, length(anchors))` — the solver behavior is independent of initial position for these unit tests (they're testing the force loop, not the initialization).

**Specifically:** wherever Task 11 inserted

```julia
psizes = [s .+ 2*Float32(params.box_padding) for s in sizes]
init = init_offsets(anchors, psizes, params)
offsets, dropped = solve_repel(anchors, sizes, init, params)
```

simplify to

```julia
init = zeros(Vec2f, length(anchors))
offsets, dropped = solve_repel(anchors, sizes, init, params)
```

This changes the numerical output of `solve_repel` in those tests because the initial offsets differ. **Rebaseline the affected expected values:**

1. Run the suite after the changes.
2. For each test that fails on a numerical comparison (e.g., `@test offsets[1] ≈ Vec2f(...)`), grep the failure output for the actual computed value and update the test.
3. For tests asserting behavioral properties (`@test all(isfinite, ...)`, `@test boxes don't overlap`, etc.), no change should be needed.

Inspect `test/test_solver.jl` for these likely-affected assertion patterns:
- Any line containing `≈ Vec2f(`
- Any line asserting specific `offsets[i]` or `dropped[i]` values

- [ ] **Step 5: Run, expect pass**

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Existing integration tests (`test/test_integration.jl`) may also need rebaselining at this point if they assert specific offset values. Apply the same methodology.

- [ ] **Step 6: Smoke-test the example**

```bash
julia --project=. examples/readme_example.jl
```

The example should run without errors and produce a PNG that visually does not have crossing leader lines. Manual eyeball check.

- [ ] **Step 7: Commit**

```bash
git add src/recipe.jl src/solver.jl test/test_solver.jl test/test_integration.jl
git commit -m "Wire Voronoi init + repair_crossings! pipeline; remove golden-angle init"
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

### Task 15: Project.toml version bump + CHANGELOG

**Files:**
- Modify: `Project.toml`
- Create: `CHANGELOG.md`

- [ ] **Step 1: Bump version**

Edit `Project.toml` line 4:

```toml
version = "0.2.0"
```

- [ ] **Step 2: Create CHANGELOG.md**

```markdown
# Changelog

## v0.2.0

Layouts are now initialized using Imhof-preferred slots within each anchor's Voronoi cell when geometry allows, and a post-solve repair pass guarantees no crossing leader lines. Existing user code runs unchanged; output positions will differ from v0.1.

### Added

- Voronoi-informed initialization (`src/voronoi.jl`, `src/init.jl`) — labels start at the highest-preference Imhof slot (TR > R > T > BR > L > BL > B > TL) that fits inside their anchor's Voronoi cell, with TR fallback when none fit.
- Crossing repair (`src/crossings.jl`) — deterministic 2-opt position-swap pass guarantees no leader-line crossings in the rendered output.
- Internal `AbstractClusterSolver` interface (`src/solvers/`) — seam for future Julia-native solver implementations.
- New dependency: `DelaunayTriangulation.jl` (≥ 1.6).

### Changed

- `solve_repel` now takes a required `initial_offsets::Vector{Vec2f}` argument (was internally computed via golden-angle initialization).
- Output offset values differ from v0.1 even for identical inputs, because both the initial positions and the post-solve crossing repair affect the final layout.

### Removed

- `init_offsets` and `_GOLDEN_ANGLE` from `src/solver.jl` (replaced by `initial_offsets` in `src/init.jl`).

## v0.1.0

Initial release. Force-directed label-repel recipe for Makie.
```

- [ ] **Step 3: Commit**

```bash
git add Project.toml CHANGELOG.md
git commit -m "Bump version to 0.2.0 and add CHANGELOG"
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

Confirm the diff scope matches the spec's File Structure section: 5 new source files, 2 new test files, 1 new CHANGELOG, modifications to Project.toml/MakieTextRepel.jl/solver.jl/connectors.jl/recipe.jl/runtests.jl/test_solver.jl/test_connectors.jl/test_integration.jl.

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
