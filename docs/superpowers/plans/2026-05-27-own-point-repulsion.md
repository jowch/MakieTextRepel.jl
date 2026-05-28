# Own-Point Repulsion & Connector Geometry Hardening — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop labels from sitting on their own data markers, and stop the leader-line jank that comes with it. Closes [#1](https://github.com/jowch/MakieTextRepel.jl/issues/1).

**Architecture:** Two-layer fix. (1) Solver: replace `explode_init` with `init_offsets`, a golden-angle spiral sized to escape each label's padded box; drop the own-anchor skip in `point_push`. (2) Connectors: `clip_to_box_edge` becomes `Union{Point2f, Nothing}`; `build_connectors` gains `point_padding` keyword for anchor trim and switches to a visible-segment-length filter. Defaults bumped: `point_padding 0.0→2.0`, `min_segment_length 5.0→2.0`. Spec at `docs/superpowers/specs/2026-05-27-own-point-repulsion-design.md`.

**Tech Stack:** Julia 1.x, Makie (recipe), GeometryBasics (`Point2f`, `Vec2f`, `Rect2f`), CairoMakie for tests, `LinearAlgebra` (`norm`). Tests use `Test.@testset`. The package depends on `TextMeasure` (unreleased).

**Run all tests:** `julia --project=. -e 'using Pkg; Pkg.test()'` from the repo root. Cold start is slow (~30s for CairoMakie compile); subsequent runs in the same session use the cache. Tests are organized into `test_geometry.jl`, `test_solver.jl`, `test_connectors.jl`, `test_measure.jl`, `test_integration.jl`, all included by `test/runtests.jl`.

---

## File overview

| File | Role | Change |
|---|---|---|
| `src/geometry.jl` | Pure AABB helpers | `clip_to_box_edge` return type → `Union{Point2f, Nothing}`; strict-inside check + `t`-clamp |
| `src/connectors.jl` | `build_connectors` (leader-line construction) | Absorb `nothing` from clip (Task 1); add `point_padding` keyword, anchor trim, visible-length filter (Task 2) |
| `src/solver.jl` | `explode_init`, `solve_repel` | Replace `explode_init` → `init_offsets` (per-label golden-angle spiral, `r_min` floor; reuses the existing `const _GOLDEN_ANGLE` defined at `src/solver.jl:19`); drop own-anchor skip on the line that today says `i == j && continue   # don't repel a label from its OWN anchor` inside the `point_push` loop |
| `src/recipe.jl` | Makie recipe | Bump `point_padding` default `0.0→2.0` and `min_segment_length` default `5.0→2.0`; thread `point_padding` into the `build_connectors` lift |
| `test/test_geometry.jl` | Geometry unit tests | Add inside/face/corner cases for `clip_to_box_edge` |
| `test/test_solver.jl` | Solver unit tests | Rename `explode_init` tests → `init_offsets`, add invariant + zero-size + cluster tests, **flip** the single-label `o1 == [Vec2f(0, 0)]` assertion |
| `test/test_connectors.jl` | Connector unit tests | Add anchor-trim, anchor-inside-box, visible-length, diagonal, fan-out tests |
| `test/test_integration.jl` | Recipe smoke + render | Add adversarial clamp case; existing demo render auto-updates |
| `examples/readme_example.jl` | README hero generator | Re-run at the end of Task 4 to refresh `assets/example.png` |

The two existing solver assertions that **invert** under this design (regression checks for the fix):
- `test/test_solver.jl:12` `@test explode_init(anchors, sizes, p) == [Vec2f(0, 0), Vec2f(0, 0)]` — distinct anchors now get non-zero offsets.
- `test/test_solver.jl:53` `@test o1 == [Vec2f(0, 0)]` (single label) — isolated label now moves under own-anchor repulsion.

---

## Task 1: Geometry — strict-inside check + minimal connector absorption

**Files:**
- Modify: `src/geometry.jl:51-61` (`clip_to_box_edge`)
- Modify: `src/connectors.jl:13-20` (add one-line `edge === nothing && continue`)
- Test: `test/test_geometry.jl`

This task is intentionally atomic across two files: the geometry contract changes from `Point2f` to `Union{Point2f, Nothing}`, and the only existing caller (`build_connectors`) needs the minimum update to absorb the new return type so the test suite keeps passing.

### Step 1.1: Write failing geometry tests

- [ ] Edit `test/test_geometry.jl`. Append the new test block **after** the existing `@testset "clamp_box_offset"` (after line 48):

```julia
@testset "clip_to_box_edge: inside-box and boundary" begin
    box = box_at(Point2f(0, 0), Vec2f(0, 0), Vec2f(4, 4))   # box: x ∈ [-2, 2], y ∈ [-2, 2]

    # strictly inside on both axes → nothing
    @test clip_to_box_edge(box, Point2f(0.5, 0.5)) === nothing

    # exact center → nothing (subsumed by strict-inside)
    @test clip_to_box_edge(box, Point2f(0, 0)) === nothing

    # target on a face (|d_x| = hw, |d_y| < hh) → valid face endpoint at t = 1
    edge = clip_to_box_edge(box, Point2f(2, 0))
    @test edge !== nothing
    @test edge ≈ Point2f(2, 0)

    # target on a corner (|d_x| = hw and |d_y| = hh) → valid corner endpoint
    edge_c = clip_to_box_edge(box, Point2f(2, 2))
    @test edge_c !== nothing
    @test edge_c ≈ Point2f(2, 2)

    # target outside on x only → near face on x, clipped (regression of existing case)
    @test clip_to_box_edge(box, Point2f(100, 0)) ≈ Point2f(2, 0)

    # target outside on both axes diagonally → corner of the box on the limiting axis
    # For (4, 4): t = min(2/4, 2/4) = 0.5, edge = (2, 2)
    @test clip_to_box_edge(box, Point2f(4, 4)) ≈ Point2f(2, 2)
end
```

### Step 1.2: Run the tests; verify they fail

- [ ] Run from repo root:

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Expected: at least one failure in the new `@testset "clip_to_box_edge: inside-box and boundary"`. Most likely the first two assertions: the current implementation returns a `Point2f` for in-box targets (going past the target), not `nothing`. The other existing tests should still pass.

### Step 1.3: Implement the new `clip_to_box_edge`

- [ ] Replace `src/geometry.jl:47-61` (the existing `clip_to_box_edge` function and its docstring) with:

```julia
"""
Point on the boundary of `box` along the ray from the box center toward `target`
(ggrepel-style connector attachment). Returns `nothing` when `target` lies
strictly inside the box on both axes — no clean segment can be drawn. A target
on a face or corner is a valid endpoint at `t = 1`.
"""
function clip_to_box_edge(box::Rect2f, target::Point2f)
    c = _center(box)
    d = target .- c
    hw = box.widths[1] / 2
    hh = box.widths[2] / 2
    # strict-inside: a target on the boundary is still a valid endpoint
    (abs(d[1]) < hw && abs(d[2]) < hh) && return nothing
    tx = d[1] == 0 ? Inf32 : hw / abs(d[1])
    ty = d[2] == 0 ? Inf32 : hh / abs(d[2])
    t = clamp(min(tx, ty), 0f0, 1f0)   # defensive; strict-inside already guards
    return Point2f(c .+ t .* d)
end
```

### Step 1.4: Absorb the new return type in `build_connectors`

- [ ] Edit `src/connectors.jl`. After line 18 (`edge = clip_to_box_edge(box, anchors[i])`), add the one-line skip so the function body becomes:

```julia
function build_connectors(anchors::Vector{Point2f}, offsets::Vector{Vec2f},
                          sizes::Vector{Vec2f}, dropped::BitVector,
                          min_len::Real, box_padding::Real)
    segs = Point2f[]
    pad = Float32(box_padding)
    for i in eachindex(anchors)
        dropped[i] && continue
        norm(offsets[i]) <= min_len && continue
        psize = sizes[i] .+ 2pad
        box = box_at(anchors[i], offsets[i], psize)
        edge = clip_to_box_edge(box, anchors[i])
        edge === nothing && continue       # anchor strictly inside padded box
        push!(segs, anchors[i], edge)
    end
    return segs
end
```

This is the only line added; everything else is unchanged.

### Step 1.5: Run all tests; verify they pass

- [ ] Run:

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Expected: all tests pass, including the new `clip_to_box_edge: inside-box and boundary` block. The smoke render at `test/output/demo.png` is regenerated; the "tiny stub poking out of the label" artifact disappears (the strict-inside check now suppresses those segments), but solver-side jank (labels still sitting on their own anchors) remains until Task 3.

### Step 1.6: Commit

- [ ] Commit:

```bash
git add src/geometry.jl src/connectors.jl test/test_geometry.jl
git commit -m "geometry: clip_to_box_edge returns nothing for inside-box targets

Strict-inside check; defensive t-clamp to [0,1]. build_connectors absorbs
the new Union{Point2f, Nothing} return with a one-line skip. No behavior
change for outside-box targets (existing test at test_geometry.jl:31 still
passes). Spec: docs/superpowers/specs/2026-05-27-own-point-repulsion-design.md"
```

---

## Task 2: Connectors — `point_padding` keyword, anchor trim, visible-length filter

**Files:**
- Modify: `src/connectors.jl` (rewrite `build_connectors`)
- Test: `test/test_connectors.jl`

This task adds the connector geometry hardening. Signature gains `point_padding` as a **keyword argument with default `0.0`** so the recipe's call site keeps compiling between this task and Task 4. The old offset-magnitude filter (`norm(offsets[i]) <= min_len`) is replaced by a visible-length filter on `‖edge − seg_start‖` at the end of the per-label loop.

### Step 2.1: Write failing connector tests

- [ ] Replace `test/test_connectors.jl` contents with:

```julia
using MakieTextRepel: build_connectors
using GeometryBasics
using LinearAlgebra

@testset "build_connectors: basic" begin
    anchors = [Point2f(0, 0), Point2f(50, 0)]
    sizes = [Vec2f(20, 10), Vec2f(20, 10)]
    dropped = falses(2)

    # label 1 moved far (offset 30 in x), label 2 not moved
    offsets = [Vec2f(30, 0), Vec2f(0, 0)]
    segs = build_connectors(anchors, offsets, sizes, dropped, 5.0, 0.0)
    # label 2 has anchor at center → suppressed by strict-inside
    # label 1: anchor at (0,0); box center at (30,0); box extends x∈[20,40], so anchor
    # outside on x. edge = (20, 0). Visible length = 20 > 5 → emitted.
    @test length(segs) == 2
    @test segs[1] == Point2f(0, 0)
    @test segs[2][1] ≈ 20.0f0

    # dropped labels produce no connector
    segs_drop = build_connectors(anchors, offsets, sizes, BitVector([true, false]), 5.0, 0.0)
    @test isempty(segs_drop)
end

@testset "build_connectors: anchor trim by point_padding" begin
    # label 1 offset 30 in x; anchor at (0,0), label center at (30,0). With
    # point_padding = 4, segment start should be 4 px in along +x: (4, 0).
    anchors = [Point2f(0, 0)]
    sizes = [Vec2f(20, 10)]
    offsets = [Vec2f(30, 0)]
    dropped = falses(1)
    segs = build_connectors(anchors, offsets, sizes, dropped, 0.0, 0.0;
                            point_padding = 4.0)
    @test length(segs) == 2
    @test segs[1] ≈ Point2f(4, 0)        # start trimmed by 4 px
    @test segs[2] ≈ Point2f(20, 0)       # end at box face (unchanged)
end

@testset "build_connectors: anchor inside padded box is suppressed (locking)" begin
    # This locks behavior already established by Task 1's strict-inside check.
    # Task 2 must preserve it.
    # Box at offset (5, 0), size (20, 10) → box x ∈ [-5, 15]. Anchor at (0, 0)
    # is strictly inside → no segment.
    anchors = [Point2f(0, 0)]
    sizes = [Vec2f(20, 10)]
    offsets = [Vec2f(5, 0)]
    dropped = falses(1)
    segs = build_connectors(anchors, offsets, sizes, dropped, 0.0, 0.0)
    @test isempty(segs)
end

@testset "build_connectors: fan-out across coincident-anchor labels" begin
    # Three labels at the same anchor with three distinct offsets. Each emits a
    # segment in a distinct direction (no uniform +x bias). We construct the
    # offsets directly here (we are testing build_connectors, not the solver).
    anchors = fill(Point2f(0, 0), 3)
    sizes = [Vec2f(20, 10), Vec2f(20, 10), Vec2f(20, 10)]
    offsets = [Vec2f(30, 0), Vec2f(-20, 25), Vec2f(0, -30)]
    dropped = falses(3)
    segs = build_connectors(anchors, offsets, sizes, dropped, 0.0, 0.0)
    @test length(segs) == 6   # three segments × two endpoints
    # Three distinct edge endpoints (segs[2], segs[4], segs[6]) — no bias.
    ends = (segs[2], segs[4], segs[6])
    @test ends[1] != ends[2] && ends[2] != ends[3] && ends[1] != ends[3]
    # Each segment direction is roughly toward its offset (sanity for no bias).
    @test segs[2][1] > 0                          # label 1 ends to the right
    @test segs[4][1] < 0 && segs[4][2] > 0        # label 2 ends upper-left
    @test segs[6][2] < 0                          # label 3 ends below
end

@testset "build_connectors: visible-length filter" begin
    # Anchor outside box but only by 1 px; with min_len = 2 the segment is
    # suppressed even though norm(offset) is large.
    # box at offset (11, 0), size (20, 10) → box x ∈ [1, 21]. Anchor at (0, 0).
    # edge = (1, 0). Visible length = 1.0 < min_len 2.0 → suppressed.
    anchors = [Point2f(0, 0)]
    sizes = [Vec2f(20, 10)]
    offsets = [Vec2f(11, 0)]
    dropped = falses(1)
    segs = build_connectors(anchors, offsets, sizes, dropped, 2.0, 0.0)
    @test isempty(segs)

    # Same setup but offset = (13, 0): box x ∈ [3, 23]. Anchor at (0, 0). Edge =
    # (3, 0). Visible length = 3.0 > 2.0 → emitted.
    segs2 = build_connectors(anchors, [Vec2f(13, 0)], sizes, dropped, 2.0, 0.0)
    @test length(segs2) == 2
    @test segs2[2] ≈ Point2f(3, 0)
end

@testset "build_connectors: diagonal offset terminates on the limiting face" begin
    # Square box, equal diagonal offset → t = min(hw/|dx|, hh/|dy|), both equal,
    # corner endpoint.
    anchors = [Point2f(0, 0)]
    sizes = [Vec2f(10, 10)]
    offsets = [Vec2f(20, 20)]   # box center at (20, 20), box [15..25] × [15..25]
    dropped = falses(1)
    segs = build_connectors(anchors, offsets, sizes, dropped, 0.0, 0.0)
    @test length(segs) == 2
    @test segs[1] == Point2f(0, 0)
    @test segs[2] ≈ Point2f(15, 15)   # near corner of the box
end

@testset "build_connectors: keyword default for point_padding is 0.0" begin
    # Passing no point_padding keyword should match passing point_padding = 0.0.
    anchors = [Point2f(0, 0)]
    sizes = [Vec2f(20, 10)]
    offsets = [Vec2f(30, 0)]
    dropped = falses(1)
    a = build_connectors(anchors, offsets, sizes, dropped, 0.0, 0.0)
    b = build_connectors(anchors, offsets, sizes, dropped, 0.0, 0.0;
                         point_padding = 0.0)
    @test a == b
end
```

### Step 2.2: Run tests; verify failures

- [ ] Run:

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Expected failures in the new connector testsets:
- `anchor trim by point_padding`: today's `build_connectors` doesn't accept the `point_padding` keyword → `MethodError` at the call site.
- `visible-length filter` setup 1 (`offset = (11, 0)`, `min_len = 2.0`): today's offset-magnitude filter (`norm(offsets[i]) <= min_len`) sees `norm = 11 > 2` and passes the filter, so a segment is emitted; the test expects empty.
- `fan-out across coincident-anchor labels`: today emits some segments but the directions are interleaved with old offset-magnitude suppression; result count or coordinates differ from expectations.
- `keyword default for point_padding is 0.0` — `MethodError` until Step 2.3 lands the keyword.

Already-green (locking) testsets, expected to pass even before Step 2.3:
- `anchor inside padded box is suppressed (locking)` — locked by Task 1's strict-inside check.
- `basic` and `diagonal` — exercise existing geometry that Task 1 already handles.

### Step 2.3: Implement the new `build_connectors`

- [ ] Replace `src/connectors.jl` contents with:

```julia
# connectors.jl — pure connector-segment construction (pixel space).

"""
Build flat `Point2f` endpoint pairs for `linesegments!` (pixel space): each
label gets a segment from `anchor + point_padding · û` (trimmed to leave a
visible gap at the data marker) to the near edge of its padded box. Segments
are suppressed when (a) the label is dropped, (b) the anchor lies strictly
inside the padded box (no clean segment), (c) the anchor-to-edge distance
is less than `point_padding` (trim would invert direction), or (d) the
visible segment length is below `min_len`.

`point_padding` is a keyword argument so the recipe can ship the connector
change before the recipe is rewired to forward the value (intermediate
builds stay green).
"""
function build_connectors(anchors::Vector{Point2f}, offsets::Vector{Vec2f},
                          sizes::Vector{Vec2f}, dropped::BitVector,
                          min_len::Real, box_padding::Real;
                          point_padding::Real = 0.0)
    segs = Point2f[]
    pad = Float32(box_padding)
    ppad = Float32(point_padding)
    min_len_f = Float32(min_len)
    for i in eachindex(anchors)
        dropped[i] && continue
        psize = sizes[i] .+ 2pad
        box = box_at(anchors[i], offsets[i], psize)
        edge = clip_to_box_edge(box, anchors[i])
        edge === nothing && continue       # anchor strictly inside padded box
        dir = edge .- anchors[i]
        dlen = norm(dir)
        dlen <= ppad && continue           # trim would invert direction
        seg_start = anchors[i] .+ (ppad / dlen) .* dir
        norm(edge .- seg_start) <= min_len_f && continue   # visible-length filter
        push!(segs, seg_start, edge)
    end
    return segs
end
```

### Step 2.4: Run tests; verify all pass

- [ ] Run:

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Expected: all tests pass. Note the integration smoke at `test_integration.jl:32-44 ("connectors render")` uses `min_segment_length = 1.0` and very-close anchors, so the segment array stays non-empty.

### Step 2.5: Commit

- [ ] Commit:

```bash
git add src/connectors.jl test/test_connectors.jl
git commit -m "connectors: add point_padding keyword, anchor trim, visible-length filter

build_connectors now (a) suppresses segments when the anchor is strictly
inside the padded box (the Task-1 invariant), (b) trims the anchor end by
point_padding so leader lines stop short of markers, and (c) filters on
the visible segment length, not the offset magnitude. point_padding is a
keyword arg with default 0.0 — the recipe still passes only box_padding,
so behavior is identical until Task 4 bumps the default. Spec section:
'Connectors (src/connectors.jl)'."
```

---

## Task 3: Solver — `init_offsets` + drop own-anchor skip

**Files:**
- Modify: `src/solver.jl` (replace `explode_init` with `init_offsets`; remove line `i == j && continue   # don't repel a label from its OWN anchor` inside the `point_push` loop)
- Test: `test/test_solver.jl`

This is the substantive solver change. Both inverting tests (`test_solver.jl:12` and `test_solver.jl:53`) are updated in this task because they encoded the bug we're fixing.

### Step 3.1: Write failing solver tests

- [ ] Open `test/test_solver.jl`. Make four changes. **Apply them in reverse file order (D → C → B → A) so earlier edits don't shift the search targets for later ones.** Each change uses a content-anchored search target so line numbers don't matter.

**Change B (do this first, despite the letter): update the top-of-file imports.** Find the line `using MakieTextRepel: RepelParams, explode_init` (currently `test_solver.jl:2`) and replace it with:

```julia
using MakieTextRepel: RepelParams, init_offsets, box_at
```

The added `box_at` import is needed by the new `init_offsets` testset's geometric-invariant loop; the existing `using MakieTextRepel: box_at` on line 25 becomes a redundant-but-harmless duplicate (leave it — minimizing diff). The `init_offsets` import works the same way `explode_init` does today: `using Module: name` brings in unexported names too, so no `export` declaration is needed.

**Change D (do this second): append a new testset at the end of the file**, after the existing `@testset "solve_repel converges under edge-crowding" begin ... end` block:

```julia
@testset "solve_repel: own-anchor repulsion fans out coincident clusters" begin
    # 5+ labels at the same anchor, varying sizes. All finite, all distinct,
    # no two boxes still overlapping after solve.
    co = fill(Point2f(0, 0), 6)
    cs = [Vec2f(20, 10), Vec2f(15, 12), Vec2f(25, 8),
          Vec2f(18, 14), Vec2f(22, 9), Vec2f(16, 11)]
    offs, _ = solve_repel(co, cs, RepelParams(box_padding = 2.0))
    @test all(o -> all(isfinite, o), offs)
    @test !_any_overlap(co, offs, cs, 2.0; tol = 0.5)
    # No two offsets identical (golden-angle init guarantees distinct seeds).
    for i in 1:length(offs), j in (i+1):length(offs)
        @test offs[i] != offs[j]
    end
end
```

**Change C (do this third): flip the single-label assertion.** Find the unique line `@test o1 == [Vec2f(0, 0)]` (currently at `test_solver.jl:53`, inside `@testset "solve_repel"`) and replace it with:

```julia
    # Single label: own-anchor repulsion now active. The spring pulls inward from
    # init but cannot reach 0; equilibrium sits inside the init radius.
    @test norm(o1[1]) > 0
    # Init magnitude for label size (10, 4) and box_padding = 0:
    # r_init = sqrt(5^2 + 2^2) ≈ 5.39. Final |offset| must be < this (spring
    # pulled inward).
    @test norm(o1[1]) < 5.4f0
```

**Change A (do this last): replace the entire `@testset "explode_init"` block.** Find the block starting `@testset "explode_init" begin` (currently `test_solver.jl:6`) and ending at its matching `end` (currently line 22) and replace the whole block (including the opening and closing lines) with:

```julia
@testset "init_offsets" begin
    p = RepelParams()

    # distinct anchors → every label gets a deterministic golden-angle offset
    anchors = [Point2f(0, 0), Point2f(100, 100)]
    sizes = [Vec2f(10, 10), Vec2f(10, 10)]   # CALLER passes PADDED sizes; here pad=0 for simplicity
    o1 = init_offsets(anchors, sizes, p)
    o2 = init_offsets(anchors, sizes, p)
    @test o1 == o2                              # deterministic
    @test norm(o1[1]) > 0 && norm(o1[2]) > 0    # every label moved off (0,0)
    @test o1[1] != o1[2]                        # distinct angles per label

    # Geometric invariant: at the init offset, the anchor lies ON or OUTSIDE the
    # (passed-in, already-padded) box of the label.
    for i in eachindex(anchors)
        box = box_at(anchors[i], o1[i], sizes[i])
        c = box.origin .+ box.widths ./ 2
        d = anchors[i] .- c
        hw, hh = box.widths[1] / 2, box.widths[2] / 2
        @test abs(d[1]) >= hw - 1e-4 || abs(d[2]) >= hh - 1e-4   # on or outside
    end

    # coincident anchors → still get distinct golden-angle offsets
    co = [Point2f(0, 0), Point2f(0, 0), Point2f(0, 0)]
    cs = [Vec2f(10, 10), Vec2f(10, 10), Vec2f(10, 10)]
    occ = init_offsets(co, cs, p)
    @test occ[1] != occ[2]
    @test occ[2] != occ[3]
    @test occ[1] != occ[3]

    # zero-size label (empty string) → r_min floor produces a non-zero offset
    zero_sz = [Vec2f(0, 0)]
    zero_anchor = [Point2f(0, 0)]
    @test norm(init_offsets(zero_anchor, zero_sz, p)[1]) >= 1.0f0 - 1e-4
end
```

### Step 3.2: Run tests; verify the expected failure pattern

- [ ] Run:

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

**Expected pattern:** the entire `test/test_solver.jl` file errors at load time because Change B's import line references `init_offsets`, which doesn't exist yet (`UndefVarError: init_offsets not defined`). Every testset in `test_solver.jl` is reported as an error — including the unchanged ones (`compute_drops`, `solve_repel clamping`, etc.). This is the correct TDD "red": the file cannot load, so the new behavior is not implemented. Tests in other files (`test_geometry.jl`, `test_connectors.jl`, `test_measure.jl`, `test_integration.jl`) still run and pass.

After Step 3.3 lands the `init_offsets` definition, the file loads, and the expected per-test failures collapse to: the new `@testset "init_offsets"` runs and passes; the flipped single-label assertion now passes; the new fan-out testset passes.

### Step 3.3: Implement `init_offsets` and drop the own-anchor skip

- [ ] Edit `src/solver.jl`. Three changes; **apply A last** so the content anchors for B and C don't move under you. All targets are content-anchored, not line-numbered.

**Change B (do this first): rename the call site.** In `solve_repel`, find the unique line `offsets = explode_init(anchors, psizes, p)` and change it to:

```julia
    offsets = init_offsets(anchors, psizes, p)
```

There is only one call to `explode_init` in the file.

**Change C (do this second): drop the own-anchor skip in the `point_push` loop.** Find the unique line `i == j && continue   # don't repel a label from its OWN anchor` (the trailing comment makes it unique within `solver.jl`; the identical `i == j && continue` line in the `overlap_push` loop two lines above has no such comment and **must not** be touched). Replace it with:

```julia
            # Own anchor is included: keeps isolated labels off their own point.
```

(Copy-paste the code block verbatim — the leading whitespace is 12 spaces to match the surrounding `for j` body, which sits inside `for it`, then `for i`, then `for j`.)

**Change A (do this last): replace the `explode_init` function and its docstring.** Find the block starting with the docstring line `"""` immediately above `function explode_init(anchors::Vector{Point2f}, sizes::Vector{Vec2f}, p::RepelParams)` and ending at the matching `end` of that function. Replace the entire block (docstring + function) with:

```julia
"""
Deterministic initial offsets. Every label gets a per-index golden-angle
direction sized to escape its own (already padded) box, so the force loop
starts with each anchor on or outside its label box. Subsumes the old
"only fan out coincident anchors" behavior. Determinism: pure function of
index and passed-in sizes.

`psizes` is the *padded* size (the caller adds `2·box_padding`); the
corner-distance computed here is therefore the corner of the padded box,
guaranteeing the anchor lies on or outside it for any spiral angle. The
`1.0f0` floor only binds for degenerate zero-size labels with zero
padding (e.g. empty strings); in normal layouts it never fires.
"""
function init_offsets(anchors::Vector{Point2f}, psizes::Vector{Vec2f}, p::RepelParams)
    n = length(anchors)
    offsets = Vector{Vec2f}(undef, n)
    for i in 1:n
        hw = psizes[i][1] / 2
        hh = psizes[i][2] / 2
        r  = max(sqrt(hw*hw + hh*hh), 1f0)
        θ  = _GOLDEN_ANGLE * Float32(i)
        offsets[i] = Vec2f(r * cos(θ), r * sin(θ))
    end
    return offsets
end
```

After Change A, the file has `init_offsets` defined and called; the own-anchor skip is gone. The overlap-push loop (which has its own `i == j && continue` without the trailing comment) is untouched — labels still don't push themselves through box-overlap.

### Step 3.4: Run tests; verify all pass

- [ ] Run:

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Expected: all tests pass. The existing determinism test inside the `solve_repel` testset (`o_a == o_b`) still passes — `init_offsets` is deterministic and the new force term is symmetric, so byte-identical output is preserved on a non-degenerate input. The clamping tests are unaffected (own-anchor push just adds another push term that the clamp absorbs).

**Most likely surprise failure:** the `@testset "solve_repel converges under edge-crowding"` block (wide labels in a small viewport). The new own-anchor force adds a term that may shift the equilibrium relative to the old `explode_init` baseline. The convergence assertion `maximum(norm.(oa .- ob)) < 1.0` should still hold (cooling drives convergence regardless of the absolute equilibrium), but if it doesn't, audit before changing it.

If any previously-passing test now fails, audit it: the most likely culprit is a numeric expectation tuned against the old `explode_init` magnitude. Update the expectation to match the new corner-distance formula. Do **not** weaken an invariant test (e.g. `_any_overlap` returning false) — those are correctness guarantees.

### Step 3.5: Commit

- [ ] Commit:

```bash
git add src/solver.jl test/test_solver.jl
git commit -m "solver: replace explode_init with init_offsets; drop own-anchor skip

Every label gets a deterministic golden-angle initial offset sized to the
corner of its padded box, so anchors start on or outside their own label
box. The own-anchor skip in the point_push loop is removed, so isolated
labels feel the same repulsion from their own point as from every other
point — equilibrium with force_pull settles them near (not on) the anchor.

Two existing tests inverted: test_solver.jl 'distinct anchors → no nudge'
and 'single label never moves' both encoded the bug being fixed. New
tests lock the geometric invariant (anchor on or outside padded box at
init), zero-size-label r_min floor, and coincident-cluster fan-out.
Closes #1. Spec section: 'Solver (src/solver.jl)'."
```

---

## Task 4: Recipe — wire `point_padding`, bump defaults, regenerate hero

**Files:**
- Modify: `src/recipe.jl` (the `@recipe` defaults block and the `build_connectors` lift)
- Add: `test/test_integration.jl` adversarial-clamp testset (after the recipe is wired)
- Re-run: `examples/readme_example.jl`

### Step 4.1: Bump defaults and wire `point_padding`

- [ ] Edit `src/recipe.jl`. Three changes; all content-anchored.

**Change A — bump `point_padding` default.** Find the two-line block:

```julia
    "Pixel halo around each data point."
    point_padding = 0.0
```

and replace it with:

```julia
    "Pixel halo around each data point — used both by the solver (repulsion halo) and by the connector layer (gap between the marker and the start of the leader line)."
    point_padding = 2.0
```

**Change B — bump `min_segment_length` default.** Find the two-line block:

```julia
    "Suppress a connector if the label moved fewer than this many pixels."
    min_segment_length = 5.0
```

and replace it with:

```julia
    "Suppress a connector if its *visible* length (anchor end trimmed by `point_padding`, label end clipped to box face) is fewer than this many pixels."
    min_segment_length = 2.0
```

**Change C — wire `point_padding` into the `build_connectors` lift.** Find the block:

```julia
    seg_points = lift(solved, p.min_segment_length, p.box_padding, p.segments) do s, ml, bp, on
        on || return Point2f[]
        build_connectors(s.anchors, s.offsets, s.sizes, s.dropped,
                         Float64(ml), Float64(bp))
    end
```

and replace it with:

```julia
    seg_points = lift(solved, p.min_segment_length, p.box_padding, p.point_padding, p.segments) do s, ml, bp, pp, on
        on || return Point2f[]
        build_connectors(s.anchors, s.offsets, s.sizes, s.dropped,
                         Float64(ml), Float64(bp); point_padding = Float64(pp))
    end
```

### Step 4.2: Run all tests; verify all pass

- [ ] Run:

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Expected: every existing test in every file passes. The `connectors render` test (`@testset "connectors render"` in `test_integration.jl`) explicitly sets `min_segment_length = 1.0` and uses very-close anchors, so it stays green regardless of the default bump.

If any test fails, the most likely cause is that the integration smoke test's `iseven(length(...))` assertion is still happy (zero is even), but a more specific assertion you added in earlier tasks may have tightened around the old default. Audit and update the expectation.

### Step 4.3: Add the adversarial-clamp locking test (post-fix invariant)

This test was deferred from earlier tasks because it depends on the full pipeline (init + own-anchor repulsion + new `point_padding` default) to be deterministic. Now that everything is wired, it's a clean locking test.

- [ ] Append to `test/test_integration.jl`:

```julia
@testset "connectors suppressed when clamp pins anchor inside its own label" begin
    # Tiny viewport + wide label anchored at the figure corner: the clamp
    # slides the padded box inward, but the anchor stays at the corner and
    # ends up inside the clamped box. The connector layer's strict-inside
    # check suppresses the segment. Locks the post-Task-4 behavior; relies on
    # the bumped default point_padding = 2.0 to ensure the clamped box is
    # large enough to contain the anchor.
    fig = Figure(size = (240, 200)); ax = Axis(fig[1, 1])
    pts = Point2f[(0, 0)]
    pl = textrepel!(ax, pts; text = ["a-very-wide-label-name"],
                    segments = true, min_segment_length = 0.0)
    Makie.update_state_before_display!(fig.scene)

    seg_plots = filter(c -> c isa LineSegments, pl.plots)
    @test length(seg_plots) == 1
    @test isempty(seg_plots[1][1][])
end
```

- [ ] Run all tests again:

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Expected: all tests pass, including the new adversarial-clamp testset.

### Step 4.4: Regenerate the README hero and inspect the smoke demo

- [ ] Regenerate the README hero:

```bash
julia --project=. examples/readme_example.jl
```

- [ ] Inspect `assets/example.png` (now tracked) and `test/output/demo.png` (regenerated by the integration tests; *not* committed — `test/output/` is in `.gitignore`):

**What to look for in `assets/example.png`** (compare against the version currently on `main`):
- `textrepel!` panel (right): every label (`node 1`–`node 22`) is offset from its tomato dot. No label should overlap or sit on top of a dot.
- A thin gray leader line connects each displaced label to its dot. The line should stop a small visible gap (≈2 px at default DPI) short of the dot, not disappear under it.
- `text!` panel (left) is unchanged — it doesn't use the recipe.

**What failure looks like:**
- Any label centered on or partially overlapping its dot → solver-side fix not effective. Re-check Task 3 (likely Change A or the own-anchor skip removal).
- Leader lines that touch or enter the dots → `point_padding` not threaded. Re-check Task 4.1 Change C (the `point_padding` argument to `build_connectors`).
- Tiny stubs poking out of label boxes → Task 1 strict-inside check not effective. Re-check Task 1.3.

**`test/output/demo.png`:** five labels (`alpha`, `beta`, `gamma`, `delta`, `epsilon`) should each be visibly off their tomato markers, with short leader lines stopping a couple of pixels short of each marker. Same failure modes as above apply.

### Step 4.5: Commit

- [ ] Commit. Do **not** add `test/output/demo.png` — it's gitignored (`test/output/` listed in `.gitignore`); it's a local artifact that CI regenerates on every run.

```bash
git add src/recipe.jl test/test_integration.jl assets/example.png
git commit -m "recipe: wire point_padding through; bump defaults; regen hero

point_padding default 0.0 → 2.0 px; min_segment_length default 5.0 → 2.0
px (re-tuned for the new visible-length filter semantics). Recipe forwards
point_padding into the build_connectors lift. Adversarial-clamp test
locks the 'connector suppressed when anchor falls inside its own clamped
box' invariant.

README hero (assets/example.png) regenerated — labels no longer sit on
their markers. (test/output/demo.png is also regenerated locally but is
gitignored.)

Closes #1."
```

---

## Self-review checklist (pre-PR)

After completing all four tasks and before opening a PR, run from the repo root:

- [ ] `julia --project=. -e 'using Pkg; Pkg.test()'` — full suite green.
- [ ] Open `assets/example.png` — `textrepel!` panel labels are all off their markers.
- [ ] Open `test/output/demo.png` — five labels off their markers.
- [ ] `git log --oneline -5` shows four focused commits matching the four tasks.
- [ ] PR description includes `Closes #1` so the issue auto-closes on merge.

---

## Notes for the implementer

- **Don't widen the `< vs ≤` choices.** The spec is explicit: strict `<` inside-box (face/corner are valid endpoints), `≤` for the visible-length and anchor-trim filters. Mismatches will fail tests.
- **`init_offsets` receives the *padded* sizes** (the solver computes `psizes` before calling it). Don't re-add `box_padding` inside `init_offsets`.
- **The own-anchor skip removal is one line.** It's the `i == j && continue   # don't repel a label from its OWN anchor` line inside the `point_push` loop — the trailing comment makes it unique. The identical `i == j && continue` line inside the `overlap_push` loop (two lines above, no trailing comment) **stays** — labels still don't push themselves through box-overlap.
- **`point_padding` is a keyword argument** on `build_connectors`. Don't promote it to positional even though the recipe always passes it after Task 4.
- **Determinism is load-bearing.** Don't add any per-iteration randomness or hash-based logic. Same input → byte-identical output.
- **If a test you didn't write fails after Task 3**, audit it before changing it. Most often it's a numeric expectation tied to the old `explode_init` magnitude. The intent (no-NaN, no-overlap, determinism, axis-lock) is preserved; only the numeric values shift.
