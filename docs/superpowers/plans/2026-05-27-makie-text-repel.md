# MakieTextRepel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A `ggrepel`/`adjustText`-style label-repel recipe for Makie/CairoMakie: `textrepel!(ax, positions; text=labels)` displaces overlapping labels and draws connector lines.

**Architecture:** Three isolated layers — **measure** (pixel box sizes via TextMeasure.jl for plain text, Makie `full_boundingbox` for rich text), **solve** (a pure-Julia deterministic force-directed solver in pixel space, no Makie types), **render** (a Makie recipe that projects anchors to pixels, calls the solver, and draws `text!`/boxes + connectors). Static output, seamed so a reactive wrapper is a later add-on.

**Tech Stack:** Julia 1.11+, Makie 0.24 / CairoMakie 0.15, TextMeasure.jl (unregistered, via `[sources]`), GeometryBasics, LinearAlgebra.

---

## File Structure

| File | Responsibility |
|---|---|
| `Project.toml` | Metadata, deps, `[sources]` for TextMeasure, compat, test target |
| `src/MakieTextRepel.jl` | Module: includes, exports (`textrepel`, `textrepel!`) |
| `src/geometry.jl` | Pure AABB helpers: `box_at`, `overlap_push`, `point_push`, `clip_to_box_edge` (GeometryBasics only) |
| `src/solver.jl` | Pure solver: `RepelParams`, `explode_init`, `solve_repel`, `compute_drops` (no Makie) |
| `src/connectors.jl` | Pure: `build_connectors` → pixel-space segment endpoints |
| `src/measure.jl` | Measurement layer: `measure_labels` dispatching String→TextMeasure, RichText→Makie |
| `src/recipe.jl` | `@recipe TextRepel`, `convert_arguments`, `Makie.plot!` (project→measure→solve→render) |
| `test/runtests.jl` | Includes all test files |
| `test/test_geometry.jl` | Pure geometry unit tests |
| `test/test_solver.jl` | Solver property tests (no-overlap, determinism, axis, stability) |
| `test/test_connectors.jl` | Connector building tests |
| `test/test_measure.jl` | Measurement vs Makie (needs CairoMakie) |
| `test/test_integration.jl` | End-to-end `textrepel!` smoke + reference image (CairoMakie) |

**Why these boundaries:** the pure layers (`geometry`, `solver`, `connectors`) carry zero Makie dependency in their source so they're trivially unit-testable and a reactive wrapper can re-call `solve_repel` unchanged. `measure` is the only file touching both TextMeasure and Makie internals. `recipe` is the only file with the Makie recipe machinery.

---

## Task 1: Package skeleton, dependencies, and `[sources]`

**Files:**
- Modify: `Project.toml`
- Modify: `src/MakieTextRepel.jl`
- Create: `test/runtests.jl`

- [ ] **Step 1: Write `Project.toml`**

Replace the file contents with:

```toml
name = "MakieTextRepel"
uuid = "2348ae4b-e21f-48c0-a77f-52990745b802"
authors = ["Jonathan Chen <jwhc@ucla.edu>"]
version = "0.1.0"

[deps]
GeometryBasics = "5c1252a2-5f33-56bf-86c9-59e7332b4326"
LinearAlgebra = "37e2e46d-f89d-539d-b4ee-838fcccc9c8e"
Makie = "ee78f7c6-11fb-53f2-987a-cfe4a2b5a57a"
TextMeasure = "06791c1d-2336-41e1-bd6f-a74c63395da6"

[sources]
TextMeasure = {url = "https://github.com/jowch/TextMeasure.jl", rev = "main"}

[compat]
Makie = "0.24"
julia = "1.11"

[extras]
CairoMakie = "13f3f980-e62b-5c42-98c6-ff1f3baf88f0"
Test = "8dfed614-e22c-5e08-85e1-65c5234f0b40"

[targets]
test = ["Test", "CairoMakie"]
```

- [ ] **Step 2: For local development, dev TextMeasure into the env**

Run (so the local checkout is used instead of the git URL during dev):

```bash
cd /home/jonathanchen/projects/MakieTextRepel.jl
julia --project=. -e 'using Pkg; Pkg.develop(path="/home/jonathanchen/projects/TextMeasure.jl"); Pkg.instantiate()'
```

Expected: resolves and instantiates, showing `+ TextMeasure v0.1.0 ~/projects/TextMeasure.jl`. After this, set the resolved `GeometryBasics` compat to match what Makie pulled in:

```bash
julia --project=. -e 'using Pkg; foreach(p -> p.name=="GeometryBasics" && println("GeometryBasics resolved to ", p.version), values(Pkg.dependencies()))'
```

Add a `GeometryBasics = "<major.minor>"` line to `[compat]` matching the printed version.

- [ ] **Step 3: Write the module stub**

Replace `src/MakieTextRepel.jl` with:

```julia
module MakieTextRepel

using Makie
using GeometryBasics
using LinearAlgebra
import TextMeasure

export textrepel, textrepel!

include("geometry.jl")
include("solver.jl")
include("connectors.jl")
include("measure.jl")
include("recipe.jl")

end # module MakieTextRepel
```

This will fail to load until the included files exist; we create them empty next so the package loads.

- [ ] **Step 4: Create empty include targets so the package loads**

Create each of `src/geometry.jl`, `src/solver.jl`, `src/connectors.jl`, `src/measure.jl`, `src/recipe.jl` containing only a comment line (e.g. `# geometry.jl`). This lets `using MakieTextRepel` succeed before we fill them in.

- [ ] **Step 5: Write `test/runtests.jl`**

```julia
using MakieTextRepel
using Test

@testset "MakieTextRepel.jl" begin
    include("test_geometry.jl")
    include("test_solver.jl")
    include("test_connectors.jl")
    include("test_measure.jl")
    include("test_integration.jl")
end
```

Create empty `test/test_geometry.jl`, `test/test_solver.jl`, `test/test_connectors.jl`, `test/test_measure.jl`, `test/test_integration.jl` (a comment line each) so the includes resolve.

- [ ] **Step 6: Verify the package loads and the (empty) test suite runs**

Run:

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Expected: PASS with `Test Summary: | No tests` (0 tests, no errors). The package compiled and loaded.

- [ ] **Step 7: Commit**

```bash
git add Project.toml src/ test/
git commit -m "Add package skeleton, deps, and [sources] for TextMeasure"
```

---

## Task 2: Pure geometry primitives

**Files:**
- Modify: `src/geometry.jl`
- Modify: `test/test_geometry.jl`

- [ ] **Step 1: Write failing tests**

Put in `test/test_geometry.jl`:

```julia
using MakieTextRepel: box_at, overlap_push, point_push, clip_to_box_edge
using GeometryBasics

@testset "geometry" begin
    # box_at centers a size on anchor+offset
    b = box_at(Point2f(10, 10), Vec2f(2, 0), Vec2f(4, 6))
    @test b.origin ≈ Point2f(10, 7)        # (12,10) - (2,3)
    @test b.widths ≈ Vec2f(4, 6)

    # overlap_push: non-overlapping boxes return zero
    a = box_at(Point2f(0, 0), Vec2f(0, 0), Vec2f(2, 2))
    far = box_at(Point2f(10, 0), Vec2f(0, 0), Vec2f(2, 2))
    @test overlap_push(a, far) == Vec2f(0, 0)

    # overlap_push: overlapping boxes push a away from b on the overlapping axes
    near = box_at(Point2f(1, 0), Vec2f(0, 0), Vec2f(2, 2))  # overlaps a by 1 in x
    push = overlap_push(a, near)
    @test push[1] < 0          # a is left of near, pushed further left
    @test abs(push[1]) ≈ 1.0   # overlap extent on x
    @test push[2] == 0         # boxes share y (aligned axis) → no y push

    # point_push: point outside box returns zero
    box = box_at(Point2f(0, 0), Vec2f(0, 0), Vec2f(4, 4))
    @test point_push(box, Point2f(10, 10), 0f0) == Vec2f(0, 0)

    # point_push: point inside box pushes box away from point
    pp = point_push(box, Point2f(1, 0), 0f0)
    @test pp[1] < 0            # point right-of-center → box pushed left

    # clip_to_box_edge: point on the box boundary toward the target
    edge = clip_to_box_edge(box, Point2f(100, 0))   # target far to the right
    @test edge ≈ Point2f(2, 0)                       # right edge at x=+2
end
```

- [ ] **Step 2: Run to verify failure**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL — `UndefVarError: box_at not defined`.

- [ ] **Step 3: Implement `src/geometry.jl`**

```julia
# geometry.jl — pure axis-aligned bounding-box helpers (GeometryBasics only).

"""Sign that never returns 0 (deterministic tie-break toward +)."""
sign0(x::Real) = x >= 0 ? 1f0 : -1f0

"""Box for a label of `size` (w,h) centered at `anchor + offset`."""
box_at(anchor::Point2f, offset::Vec2f, size::Vec2f) =
    Rect2f(Point2f(anchor .+ offset .- size ./ 2), size)

_center(b::Rect2f) = Point2f(b.origin .+ b.widths ./ 2)

"""
Per-axis separation push given center-difference `d` and per-axis overlaps.
Zero if not overlapping. Pushes only along axes carrying directional info; a
perfectly-aligned axis (`d[k] == 0`) contributes 0 so an aligned pair separates
along the *other* axis instead of running away together. Fully-coincident
centers fall back to +x (explosion init normally prevents this case).
"""
function _aniso_push(d, ox::Real, oy::Real)
    (ox <= 0 || oy <= 0) && return Vec2f(0, 0)
    (d[1] == 0 && d[2] == 0) && return Vec2f(ox, 0)
    px = d[1] == 0 ? 0f0 : sign0(d[1]) * ox
    py = d[2] == 0 ? 0f0 : sign0(d[2]) * oy
    return Vec2f(px, py)
end

"""Per-axis push moving box `a` away from overlapping box `b` (zero if disjoint)."""
function overlap_push(a::Rect2f, b::Rect2f)
    d = _center(a) .- _center(b)
    ox = (a.widths[1] + b.widths[1]) / 2 - abs(d[1])
    oy = (a.widths[2] + b.widths[2]) / 2 - abs(d[2])
    return _aniso_push(d, ox, oy)
end

"""
Push box away from point `p` if `p` lies within the box expanded by `padding`.
Zero vector otherwise. Uses the same aligned-axis-safe scheme as `overlap_push`.
"""
function point_push(box::Rect2f, p::Point2f, padding::Float32)
    ex = Rect2f(Point2f(box.origin .- padding), box.widths .+ 2padding)
    d = _center(ex) .- p
    ox = ex.widths[1] / 2 - abs(d[1])
    oy = ex.widths[2] / 2 - abs(d[2])
    return _aniso_push(d, ox, oy)
end

"""
Point on the boundary of `box` along the ray from the box center toward `target`
(ggrepel-style connector attachment). Returns the center if `target` == center.
"""
function clip_to_box_edge(box::Rect2f, target::Point2f)
    c = _center(box)
    d = target .- c
    (d[1] == 0 && d[2] == 0) && return c
    hw = box.widths[1] / 2
    hh = box.widths[2] / 2
    tx = d[1] == 0 ? Inf32 : hw / abs(d[1])
    ty = d[2] == 0 ? Inf32 : hh / abs(d[2])
    t = min(tx, ty)
    return Point2f(c .+ t .* d)
end
```

- [ ] **Step 4: Run to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS (geometry testset green).

- [ ] **Step 5: Commit**

```bash
git add src/geometry.jl test/test_geometry.jl
git commit -m "Add pure AABB geometry primitives"
```

---

## Task 3: Solver parameters and deterministic explosion init

**Files:**
- Modify: `src/solver.jl`
- Modify: `test/test_solver.jl`

- [ ] **Step 1: Write failing tests**

Put in `test/test_solver.jl`:

```julia
using MakieTextRepel: RepelParams, explode_init
using GeometryBasics
using LinearAlgebra

@testset "explode_init" begin
    p = RepelParams()

    # distinct anchors → no initial nudge
    anchors = [Point2f(0, 0), Point2f(100, 100)]
    sizes = [Vec2f(10, 10), Vec2f(10, 10)]
    @test explode_init(anchors, sizes, p) == [Vec2f(0, 0), Vec2f(0, 0)]

    # coincident anchors → later ones nudged off-origin, deterministically
    co = [Point2f(0, 0), Point2f(0, 0), Point2f(0, 0)]
    cs = [Vec2f(10, 10), Vec2f(10, 10), Vec2f(10, 10)]
    off1 = explode_init(co, cs, p)
    off2 = explode_init(co, cs, p)
    @test off1 == off2                 # deterministic
    @test norm(off1[2]) > 0            # second coincident label nudged
    @test norm(off1[3]) > 0
end
```

- [ ] **Step 2: Run to verify failure**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL — `UndefVarError: RepelParams not defined`.

- [ ] **Step 3: Implement params + explosion in `src/solver.jl`**

```julia
# solver.jl — pure deterministic force-directed label repel (no Makie types).

"""Solver parameters. All distances in pixels."""
Base.@kwdef struct RepelParams
    force::NTuple{2,Float64}        = (1.0, 1.0)
    force_point::NTuple{2,Float64}  = (1.0, 1.0)
    force_pull::NTuple{2,Float64}   = (0.01, 0.01)
    max_iter::Int                   = 2000
    only_move::Symbol               = :both        # :both | :x | :y
    box_padding::Float64            = 4.0
    point_padding::Float64          = 0.0
    max_overlaps::Float64           = Inf
    step_max::Float64               = 10.0          # per-iteration px clamp
    pull_threshold::Float64         = 1.0           # px; suppress spring within this
    tol::Float64                    = 0.1           # convergence: max move < tol
end

const _GOLDEN_ANGLE = Float32(π * (3 - sqrt(5)))

"""
Deterministic initial offsets. Labels whose anchor coincides with an earlier
anchor are fanned out along a golden-angle spiral so the force loop has a
non-zero gradient to act on (replaces upstream random jitter).
"""
function explode_init(anchors::Vector{Point2f}, sizes::Vector{Vec2f}, p::RepelParams)
    n = length(anchors)
    offsets = fill(Vec2f(0, 0), n)
    # NOTE: nested loops (not `for i in 1:n, j in ...`) so `break` exits only the
    # inner j-loop; a fused loop's `break` would skip all remaining i.
    for i in 1:n
        for j in 1:(i - 1)
            if norm(anchors[i] .- anchors[j]) < 1f-3
                θ = _GOLDEN_ANGLE * i
                r = (sizes[i][1] + sizes[i][2]) / 4
                offsets[i] = offsets[i] .+ Vec2f(r * cos(θ), r * sin(θ))
                break
            end
        end
    end
    return offsets
end
```

- [ ] **Step 4: Run to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS (explode_init testset green).

- [ ] **Step 5: Commit**

```bash
git add src/solver.jl test/test_solver.jl
git commit -m "Add solver params and deterministic explosion init"
```

---

## Task 4: Solver main loop

**Files:**
- Modify: `src/solver.jl`
- Modify: `test/test_solver.jl`

- [ ] **Step 1: Write failing tests**

Append to `test/test_solver.jl`:

```julia
using MakieTextRepel: solve_repel
using MakieTextRepel: box_at

# direct AABB overlap test (px), with a tolerance — the spring leaves sub-pixel
# residual overlap at equilibrium, which we treat as "separated".
function _boxes_overlap(b1, b2, tol)
    c1 = b1.origin .+ b1.widths ./ 2
    c2 = b2.origin .+ b2.widths ./ 2
    ox = (b1.widths[1] + b2.widths[1]) / 2 - abs(c1[1] - c2[1])
    oy = (b1.widths[2] + b2.widths[2]) / 2 - abs(c1[2] - c2[2])
    return ox > tol && oy > tol
end

function _any_overlap(anchors, offsets, sizes, pad; tol = 0.5)
    psizes = [s .+ Vec2f(2f0 * Float32(pad)) for s in sizes]
    boxes = [box_at(anchors[i], offsets[i], psizes[i]) for i in eachindex(anchors)]
    for i in eachindex(boxes), j in eachindex(boxes)
        i < j || continue
        _boxes_overlap(boxes[i], boxes[j], tol) && return true
    end
    return false
end

@testset "solve_repel" begin
    p = RepelParams(box_padding = 0.0)

    # empty / single
    @test solve_repel(Point2f[], Vec2f[], p) == (Vec2f[], falses(0))
    o1, d1 = solve_repel([Point2f(5, 5)], [Vec2f(10, 4)], p)
    @test o1 == [Vec2f(0, 0)]          # single label never moves
    @test d1 == falses(1)

    # two overlapping labels separate
    anchors = [Point2f(0, 0), Point2f(2, 0)]
    sizes = [Vec2f(20, 10), Vec2f(20, 10)]
    offsets, dropped = solve_repel(anchors, sizes, p)
    @test !_any_overlap(anchors, offsets, sizes, p.box_padding)
    @test !any(dropped)

    # determinism: identical inputs → identical outputs
    o_a, _ = solve_repel(anchors, sizes, p)
    o_b, _ = solve_repel(anchors, sizes, p)
    @test o_a == o_b

    # axis constraint: only_move = :x → zero y displacement
    px = RepelParams(box_padding = 0.0, only_move = :x)
    ox, _ = solve_repel(anchors, sizes, px)
    @test all(o -> o[2] == 0, ox)

    # stability: many coincident labels don't NaN
    co = fill(Point2f(0, 0), 8)
    cs = fill(Vec2f(15, 8), 8)
    oc, _ = solve_repel(co, cs, RepelParams())
    @test all(o -> all(isfinite, o), oc)
end
```

- [ ] **Step 2: Run to verify failure**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL — `UndefVarError: solve_repel not defined`.

- [ ] **Step 3: Implement the loop in `src/solver.jl`**

Append:

```julia
_clamp_step(d::Vec2f, m::Float32) = (n = norm(d); (n == 0 || n <= m) ? d : d .* (m / n))

_constrain(d::Vec2f, mode::Symbol) =
    mode === :x ? Vec2f(d[1], 0) : mode === :y ? Vec2f(0, d[2]) : d

"""
Solve label offsets (pixels) so boxes avoid each other and their anchor points.

Returns `(offsets::Vector{Vec2f}, dropped::BitVector)`. `anchors` and `sizes`
are in pixels; a label's box is centered at `anchor + offset`, padded by
`params.box_padding`.
"""
function solve_repel(anchors::Vector{Point2f}, sizes::Vector{Vec2f}, p::RepelParams)
    n = length(anchors)
    n == 0 && return (Vec2f[], falses(0))
    @assert length(sizes) == n "anchors and sizes must have equal length"

    psizes = [s .+ 2 * Float32(p.box_padding) for s in sizes]
    offsets = explode_init(anchors, psizes, p)

    fx, fy   = Float32.(p.force)
    ppx, ppy = Float32.(p.force_point)
    plx, ply = Float32.(p.force_pull)
    pad      = Float32(p.point_padding)
    smax     = Float32(p.step_max)
    pthr     = Float32(p.pull_threshold)

    for _ in 1:p.max_iter
        boxes = [box_at(anchors[i], offsets[i], psizes[i]) for i in 1:n]
        Δ = Vector{Vec2f}(undef, n)
        for i in 1:n
            f = Vec2f(0, 0)
            for j in 1:n
                i == j && continue
                push = overlap_push(boxes[i], boxes[j])
                f = f .+ Vec2f(push[1] * fx, push[2] * fy)
            end
            for j in 1:n
                i == j && continue   # don't repel a label from its OWN anchor
                pp = point_push(boxes[i], anchors[j], pad)
                f = f .+ Vec2f(pp[1] * ppx, pp[2] * ppy)
            end
            off = offsets[i]
            if norm(off) > pthr
                f = f .- Vec2f(off[1] * plx, off[2] * ply)
            end
            Δ[i] = f
        end
        maxmove = 0f0
        for i in 1:n
            d = _constrain(_clamp_step(Δ[i], smax), p.only_move)
            offsets[i] = offsets[i] .+ d
            maxmove = max(maxmove, norm(d))
        end
        maxmove < p.tol && break
    end

    return (offsets, compute_drops(anchors, offsets, psizes, p.max_overlaps))
end

# defined in Task 5; declared here so solve_repel resolves
function compute_drops end
```

Note: `compute_drops` is implemented in Task 5. To keep this task runnable, add a temporary minimal definition just below the `function compute_drops end` line:

```julia
compute_drops(anchors, offsets, psizes, max_overlaps) = falses(length(anchors))
```

(Task 5 replaces this stub with the real implementation and its tests.)

- [ ] **Step 4: Run to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS (solve_repel testset green).

- [ ] **Step 5: Commit**

```bash
git add src/solver.jl test/test_solver.jl
git commit -m "Add force-directed solver main loop"
```

---

## Task 5: Dropping policy

**Files:**
- Modify: `src/solver.jl`
- Modify: `test/test_solver.jl`

- [ ] **Step 1: Write failing tests**

Append to `test/test_solver.jl`:

```julia
using MakieTextRepel: compute_drops

@testset "compute_drops" begin
    anchors = [Point2f(0, 0), Point2f(1, 0), Point2f(2, 0)]
    # Narrow boxes (width 1.5): neighbours 1px apart overlap; the ends, 2px
    # apart, do NOT. (Wide boxes would make all three mutually overlap.)
    psizes = [Vec2f(1.5, 1.0), Vec2f(1.5, 1.0), Vec2f(1.5, 1.0)]
    offsets = [Vec2f(0, 0), Vec2f(0, 0), Vec2f(0, 0)]

    # Inf max_overlaps → nothing dropped
    @test compute_drops(anchors, offsets, psizes, Inf) == falses(3)

    # max_overlaps = 1 → the middle box overlaps both neighbours (count 2) and is
    # dropped; each end overlaps only the middle (count 1) and survives.
    dropped = compute_drops(anchors, offsets, psizes, 1)
    @test dropped == BitVector([false, true, false])
end
```

- [ ] **Step 2: Run to verify failure**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL — the middle-box assertion fails (stub drops nothing).

- [ ] **Step 3: Replace the stub in `src/solver.jl`**

Delete the temporary `compute_drops(...) = falses(length(anchors))` line and the `function compute_drops end` forward declaration, and add the real implementation (place it above `solve_repel` so it's defined before use):

```julia
"""
Mark labels whose final box overlaps more than `max_overlaps` other label boxes.
`Inf` keeps everything.
"""
function compute_drops(anchors::Vector{Point2f}, offsets::Vector{Vec2f},
                       psizes::Vector{Vec2f}, max_overlaps::Real)
    n = length(anchors)
    dropped = falses(n)
    isinf(max_overlaps) && return dropped
    boxes = [box_at(anchors[i], offsets[i], psizes[i]) for i in 1:n]
    for i in 1:n
        count = 0
        for j in 1:n
            i == j && continue
            overlap_push(boxes[i], boxes[j]) != Vec2f(0, 0) && (count += 1)
        end
        dropped[i] = count > max_overlaps
    end
    return dropped
end
```

- [ ] **Step 4: Run to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS (compute_drops testset green; solve_repel still green).

- [ ] **Step 5: Commit**

```bash
git add src/solver.jl test/test_solver.jl
git commit -m "Add overlap-count dropping policy"
```

---

## Task 6: Connector building

**Files:**
- Modify: `src/connectors.jl`
- Modify: `test/test_connectors.jl`

- [ ] **Step 1: Write failing tests**

Put in `test/test_connectors.jl`:

```julia
using MakieTextRepel: build_connectors
using GeometryBasics
using LinearAlgebra

@testset "build_connectors" begin
    anchors = [Point2f(0, 0), Point2f(50, 0)]
    sizes = [Vec2f(20, 10), Vec2f(20, 10)]
    dropped = falses(2)

    # label 1 moved far (offset 30 in x), label 2 not moved
    offsets = [Vec2f(30, 0), Vec2f(0, 0)]
    segs = build_connectors(anchors, offsets, sizes, dropped, 5.0, 0.0)
    @test length(segs) == 2                       # one segment = 2 endpoints
    @test segs[1] == Point2f(0, 0)                # starts at anchor 1
    @test segs[2][1] < 30 && segs[2][1] > 0       # ends on the box's near edge

    # min_segment_length suppresses the short one
    segs2 = build_connectors(anchors, [Vec2f(2, 0), Vec2f(0, 0)], sizes, dropped, 5.0, 0.0)
    @test isempty(segs2)

    # dropped labels produce no connector
    segs3 = build_connectors(anchors, offsets, sizes, BitVector([true, false]), 5.0, 0.0)
    @test isempty(segs3)
end
```

- [ ] **Step 2: Run to verify failure**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL — `UndefVarError: build_connectors not defined`.

- [ ] **Step 3: Implement `src/connectors.jl`**

```julia
# connectors.jl — pure connector-segment construction (pixel space).

"""
Build flat `Point2f` endpoint pairs for `linesegments!` (pixel space): each kept
label that moved more than `min_len` gets a segment from its anchor to the near
edge of its (padded) box. Dropped or barely-moved labels are skipped.
"""
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
        push!(segs, anchors[i], edge)
    end
    return segs
end
```

- [ ] **Step 4: Run to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS (build_connectors testset green).

- [ ] **Step 5: Commit**

```bash
git add src/connectors.jl test/test_connectors.jl
git commit -m "Add connector segment construction"
```

---

## Task 7: Measurement layer

**Files:**
- Modify: `src/measure.jl`
- Modify: `test/test_measure.jl`

**Background:** validated facts — TextMeasure's `MakieBackend(; font, fontsize, px_per_unit)` gives boxes float-identical to Makie at `px_per_unit=1`; `Makie.text_bb` is **plain-string only** (errors on `RichText`); rich text must use `full_boundingbox(plot, :pixel)` after a cheap render. `MakieBackend` needs the font resolved to a `String`/`FTFont` (rejects `Symbol`s).

- [ ] **Step 1: Write failing tests**

Put in `test/test_measure.jl` (these need CairoMakie, which is in the test target):

```julia
using CairoMakie   # provides a Makie backend so rendering paths work
using MakieTextRepel: measure_labels
using GeometryBasics

@testset "measure_labels" begin
    font = "TeX Gyre Heros Makie"

    # plain strings: matches Makie.text_bb to floating point
    sizes = measure_labels(["Hi", "Mauna Kea"], font, 24.0, 1.0)
    @test length(sizes) == 2
    for (s, str) in zip(sizes, ["Hi", "Mauna Kea"])
        bb = Makie.text_bb(str, Makie.to_font(font), 24f0)
        @test s[1] ≈ Makie.widths(bb)[1] atol = 1e-3
        @test s[2] ≈ Makie.widths(bb)[2] atol = 1e-3
    end

    # rich text: returns a finite, positive box
    rsize = only(measure_labels([rich("H", subscript("2"), "O")], font, 24.0, 1.0))
    @test all(isfinite, rsize)
    @test rsize[1] > 0 && rsize[2] > 0
end
```

- [ ] **Step 2: Run to verify failure**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL — `UndefVarError: measure_labels not defined`.

- [ ] **Step 3: Implement `src/measure.jl`**

```julia
# measure.jl — pixel box sizes for labels. Plain text via TextMeasure (render-free);
# rich text via Makie's full_boundingbox after a cheap render.

"""Resolve a Makie font attribute to something TextMeasure/`text_bb` accept."""
_resolve_font(f) = Makie.to_font(f)
_resolve_font(f::Symbol) = Makie.to_font(String(f))

"""Measure every label, returning a `Vector{Vec2f}` of (width, height) in pixels."""
measure_labels(labels, font, fontsize::Real, px_per_unit::Real) =
    [measure_one(lbl, font, Float64(fontsize), Float64(px_per_unit)) for lbl in labels]

# Plain strings (and LaTeXString) → TextMeasure, render-free.
function measure_one(label::AbstractString, font, fontsize::Float64, ppu::Float64)
    f = _resolve_font(font)
    backend = TextMeasure.MakieBackend(; font = f, fontsize = fontsize, px_per_unit = ppu)
    lay = TextMeasure.layout(TextMeasure.prepare(backend, String(label)))
    return Vec2f(lay.size[1], lay.size[2])
end

# Rich text (and any non-string) → cheap render + full_boundingbox(:pixel).
function measure_one(label, font, fontsize::Float64, ppu::Float64)
    f = _resolve_font(font)
    scene = Scene(size = (10, 10))
    t = text!(scene, Point2f(0, 0); text = label, font = f, fontsize = Float32(fontsize))
    Makie.update_state_before_display!(scene)
    bb = Makie.full_boundingbox(t, :pixel)
    w = Makie.widths(bb)
    return Vec2f(w[1] * ppu, w[2] * ppu)
end
```

Note (deferred, out of scope for v1): the spec's glyph-coverage fallback (route plain strings through `text_bb` when the resolved font lacks the label's glyphs) is intentionally not implemented here — it's a narrow mismatched-font case. Leave a `# TODO(glyph-fallback)` comment on `measure_one(::AbstractString, …)` so it's findable.

- [ ] **Step 4: Run to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS (measure_labels testset green). First run compiles CairoMakie — may take a minute.

- [ ] **Step 5: Commit**

```bash
git add src/measure.jl test/test_measure.jl
git commit -m "Add measurement layer (TextMeasure + rich-text fallback)"
```

---

## Task 8: Recipe declaration and argument conversion

**Files:**
- Modify: `src/recipe.jl`
- Modify: `test/test_integration.jl`

- [ ] **Step 1: Write failing tests**

Put in `test/test_integration.jl`:

```julia
using CairoMakie
using MakieTextRepel

@testset "recipe declaration" begin
    # the recipe type and functions exist
    @test isdefined(MakieTextRepel, :TextRepel)
    @test isa(textrepel!, Function)

    # (xs, ys) convenience form converts to a positions vector without error
    fig = Figure()
    ax = Axis(fig[1, 1])
    pl = textrepel!(ax, [1.0, 2.0, 3.0], [1.0, 2.0, 3.0];
                    text = ["a", "b", "c"])
    @test pl isa MakieTextRepel.TextRepel
end
```

- [ ] **Step 2: Run to verify failure**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL — `TextRepel` not defined.

- [ ] **Step 3: Implement the recipe shell in `src/recipe.jl`**

Declare the recipe and the `(xs, ys)` conversion. The `plot!` body is filled in Task 9 (start with a minimal body so it constructs).

```julia
# recipe.jl — the TextRepel Makie recipe.

@recipe TextRepel (positions,) begin
    "Labels to place: Vector of String / LaTeXString / rich text."
    text = nothing

    # ── Physics (pixel space, anisotropic) ──
    "Label↔label repulsion strength (x, y)."
    force = (1.0, 1.0)
    "Label↔data-point repulsion strength (x, y)."
    force_point = (1.0, 1.0)
    "Spring pull back to the anchor (x, y)."
    force_pull = (0.01, 0.01)
    "Maximum solver iterations."
    max_iter = 2000
    "Constrain movement: :both, :x, or :y."
    only_move = :both

    # ── Spacing / dropping ──
    "Pixels of padding around each label box for the solver."
    box_padding = 4.0
    "Pixel halo around each data point."
    point_padding = 0.0
    "Drop labels overlapping more than this many others (Inf = keep all)."
    max_overlaps = Inf

    # ── Connector segments ──
    "Draw connector lines from points to displaced labels."
    segments = true
    "Suppress a connector if the label moved fewer than this many pixels."
    min_segment_length = 5.0
    "Connector color."
    segmentcolor = :gray60
    "Connector width."
    linewidth = 1.0

    # ── Background box (geom_label_repel style) ──
    "Draw a filled background box behind each label."
    background = false
    "Background fill color."
    backgroundcolor = (:white, 0.8)
    "Background stroke color."
    strokecolor = :gray70
    "Background stroke width."
    strokewidth = 0.0
    "Background corner radius (px)."
    cornerradius = 4.0

    # ── Text passthrough ──
    fontsize = @inherit fontsize
    font = @inherit font
    color = @inherit textcolor
    align = (:center, :center)
end

# (xs, ys) → Vector{Point2f}
Makie.convert_arguments(::Type{<:TextRepel}, x::AbstractVector{<:Real}, y::AbstractVector{<:Real}) =
    (Point2f.(x, y),)

function Makie.plot!(p::TextRepel)
    # Minimal body; fully wired in Task 9.
    return p
end
```

- [ ] **Step 4: Run to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS (recipe declaration testset green).

- [ ] **Step 5: Commit**

```bash
git add src/recipe.jl test/test_integration.jl
git commit -m "Add TextRepel recipe declaration and argument conversion"
```

---

## Task 9: Wire `plot!` — project, measure, solve, render text

**Files:**
- Modify: `src/recipe.jl`
- Modify: `test/test_integration.jl`

- [ ] **Step 1: Write failing tests**

Append to `test/test_integration.jl`:

```julia
@testset "textrepel end-to-end (plain text)" begin
    fig = Figure(size = (400, 400))
    ax = Axis(fig[1, 1])
    pts = Point2f[(1, 1), (1.01, 1.01), (1.02, 0.99)]   # nearly coincident
    pl = textrepel!(ax, pts; text = ["alpha", "beta", "gamma"])

    # force the scene to lay out so the recipe's compute graph runs
    Makie.update_state_before_display!(fig.scene)

    offs = pl.computed_offsets[]
    @test length(offs) == 3
    @test all(o -> all(isfinite, o), offs)
    @test maximum(norm, offs) > 0          # crowded labels actually moved
end
```

- [ ] **Step 2: Run to verify failure**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL — `pl.computed_offsets` does not exist.

- [ ] **Step 3: Implement the full `plot!` body**

Replace the minimal `Makie.plot!(p::TextRepel)` from Task 8 with:

```julia
function Makie.plot!(p::TextRepel)
    # 1. Project anchors data → scene-local pixels (compute-graph node).
    register_projected_positions!(p, Point3f;
        input_space = :data, output_space = :pixel,
        input_name = :positions, output_name = :px_anchors)

    # 2. Measure + solve. Recomputes when anchors/text/font/size or params change.
    solved = lift(p.px_anchors, p.text, p.fontsize, p.font,
                  p.force, p.force_point, p.force_pull, p.max_iter, p.only_move,
                  p.box_padding, p.point_padding, p.max_overlaps) do px, labels, fs, font,
                                                                     fr, frp, fpl, mi, om,
                                                                     bp, pp, mo
        anchors = [Point2f(q[1], q[2]) for q in px]
        sizes = measure_labels(labels, font, fs, 1.0)
        params = RepelParams(; force = Tuple(Float64.(fr)),
                               force_point = Tuple(Float64.(frp)),
                               force_pull = Tuple(Float64.(fpl)),
                               max_iter = Int(mi), only_move = Symbol(om),
                               box_padding = Float64(bp), point_padding = Float64(pp),
                               max_overlaps = Float64(mo))
        offsets, dropped = solve_repel(anchors, sizes, params)
        (; anchors, sizes, offsets, dropped)
    end

    # Expose offsets for testing / downstream use. NOTE: in Makie 0.24 `p.attributes`
    # is a ComputeGraph, not a dict — use `add_input!`, not `setindex!`.
    Makie.add_input!(p.attributes, :computed_offsets, lift(s -> s.offsets, solved))

    # 3. Render text at original DATA positions with per-label pixel offsets,
    #    filtering out dropped labels.
    keep_positions = lift(p.positions, solved) do pos, s
        Point2f[pos[i] for i in eachindex(pos) if !s.dropped[i]]
    end
    keep_text = lift(p.text, solved) do labels, s
        [labels[i] for i in eachindex(labels) if !s.dropped[i]]
    end
    keep_offsets = @lift Vec2f[$solved.offsets[i] for i in eachindex($solved.offsets) if !$solved.dropped[i]]

    text!(p, keep_positions;
        text = keep_text, offset = keep_offsets, markerspace = :pixel,
        fontsize = p.fontsize, font = p.font, color = p.color, align = p.align)

    return p
end
```

Note for the implementer: `register_projected_positions!` registers `px_anchors` as a compute-graph node read here via `p.px_anchors`. If accessing it as a plain Observable in `lift` needs adjustment for this Makie version, consult `~/.julia/packages/Makie/p9K7f/src/utilities/projection_utils.jl:49-55` and the `textlabel` recipe usage; the contract (data→pixel positions) is fixed even if the access form needs a tweak.

- [ ] **Step 4: Run to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS (end-to-end plain-text testset green).

- [ ] **Step 5: Commit**

```bash
git add src/recipe.jl test/test_integration.jl
git commit -m "Wire plot!: project, measure, solve, render text"
```

---

## Task 10: Connector rendering

**Files:**
- Modify: `src/recipe.jl`
- Modify: `test/test_integration.jl`

- [ ] **Step 1: Write failing test**

Append to `test/test_integration.jl`:

```julia
@testset "connectors render" begin
    fig = Figure(size = (400, 400))
    ax = Axis(fig[1, 1])
    pts = Point2f[(1, 1), (1.005, 1.005)]   # very close → labels must move far
    pl = textrepel!(ax, pts; text = ["overlapping one", "overlapping two"],
                    segments = true, min_segment_length = 1.0)
    Makie.update_state_before_display!(fig.scene)

    # a LineSegments child plot exists and has an even number of endpoints
    seg_plots = filter(c -> c isa LineSegments, pl.plots)
    @test length(seg_plots) == 1
    @test iseven(length(seg_plots[1][1][]))
end
```

- [ ] **Step 2: Run to verify failure**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL — no `LineSegments` child plot.

- [ ] **Step 3: Add connector rendering to `plot!`**

Insert before `return p` in `Makie.plot!(p::TextRepel)`:

```julia
    # 4. Connector segments (pixel space; coexists with data-space text anchors).
    seg_points = lift(solved, p.min_segment_length, p.box_padding, p.segments) do s, ml, bp, on
        on || return Point2f[]
        build_connectors(s.anchors, s.offsets, s.sizes, s.dropped,
                         Float64(ml), Float64(bp))
    end
    linesegments!(p, seg_points; space = :pixel,
        color = p.segmentcolor, linewidth = p.linewidth, visible = p.segments)
```

(The `space = :pixel` segments use the scene-local pixel anchors, consistent with the text's pixel offsets.)

- [ ] **Step 4: Run to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS (connectors testset green).

- [ ] **Step 5: Commit**

```bash
git add src/recipe.jl test/test_integration.jl
git commit -m "Render connector segments in pixel space"
```

---

## Task 11: Background-box rendering option

**Files:**
- Modify: `src/recipe.jl`
- Modify: `test/test_integration.jl`

- [ ] **Step 1: Write failing test**

Append to `test/test_integration.jl`:

```julia
@testset "background boxes" begin
    fig = Figure(size = (400, 400))
    ax = Axis(fig[1, 1])
    pl = textrepel!(ax, Point2f[(1, 1), (2, 2)]; text = ["one", "two"],
                    background = true)
    Makie.update_state_before_display!(fig.scene)

    # a Poly child plot exists when background = true
    poly_plots = filter(c -> c isa Poly, pl.plots)
    @test length(poly_plots) == 1
end
```

- [ ] **Step 2: Run to verify failure**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL — no `Poly` child plot.

- [ ] **Step 3: Add background rendering to `plot!`**

Insert before the `text!(...)` call in `Makie.plot!(p::TextRepel)` so boxes draw beneath the text. The boxes are pixel-space rectangles at each kept label's solved position:

```julia
    # 3a. Optional background boxes (drawn beneath text), pixel space.
    box_rects = lift(solved, p.background, p.box_padding) do s, bg, bp
        bg || return Rect2f[]
        pad = Float32(bp)
        Rect2f[box_at(s.anchors[i], s.offsets[i], s.sizes[i] .+ 2pad)
               for i in eachindex(s.anchors) if !s.dropped[i]]
    end
    poly!(p, box_rects; space = :pixel,
        color = p.backgroundcolor, strokecolor = p.strokecolor,
        strokewidth = p.strokewidth, visible = p.background)
```

Note: the recipe declares `cornerradius`; rounded corners are a polish item — for v1 plain `Rect2f` boxes are acceptable. Leave `cornerradius` wired as an attribute (unused by the plain-rect path) and a `# TODO(rounded-corners)` comment.

- [ ] **Step 4: Run to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS (background boxes testset green).

- [ ] **Step 5: Commit**

```bash
git add src/recipe.jl test/test_integration.jl
git commit -m "Add optional background-box rendering"
```

---

## Task 12: Reference image test, README, and CI

**Files:**
- Modify: `test/test_integration.jl`
- Create: `test/reference/` (generated baseline image)
- Modify: `README.md`
- Create: `.github/workflows/CI.yml`

- [ ] **Step 1: Add a visual smoke test that saves an artifact**

Append to `test/test_integration.jl`:

```julia
@testset "visual artifact renders" begin
    fig = Figure(size = (600, 400))
    ax = Axis(fig[1, 1]; title = "textrepel demo")
    pts = Point2f[(1, 1), (1.1, 1.05), (1.05, 0.9), (2, 2), (2.05, 2.02)]
    scatter!(ax, pts; color = :tomato)
    textrepel!(ax, pts; text = ["alpha", "beta", "gamma", "delta", "epsilon"],
               segments = true)
    out = joinpath(@__DIR__, "reference", "demo.png")
    mkpath(dirname(out))
    save(out, fig)
    @test isfile(out) && filesize(out) > 0
end
```

This is a render-without-error smoke test that also produces a baseline `demo.png` for manual visual inspection. (Pixel-exact reference comparison via `ReferenceTests.jl` is deferred — see note.)

- [ ] **Step 2: Run to verify it passes and produces the image**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS; `test/reference/demo.png` exists. Open it and confirm labels are separated and connectors look right.

- [ ] **Step 3: Write `README.md`**

```markdown
# MakieTextRepel.jl

A `ggrepel`/`adjustText`-style label-repel recipe for [Makie](https://docs.makie.org).
Automatically displaces overlapping text labels and draws connector lines back to
their data points.

## Installation

MakieTextRepel depends on the (currently unregistered) TextMeasure.jl. Until both are
registered:

```julia
using Pkg
Pkg.add(url="https://github.com/jowch/TextMeasure.jl")
Pkg.add(url="https://github.com/jowch/MakieTextRepel.jl")
```

## Usage

```julia
using CairoMakie, MakieTextRepel

fig = Figure()
ax = Axis(fig[1, 1])
pts = Point2f[(1, 1), (1.1, 1.05), (1.05, 0.9)]
scatter!(ax, pts)
textrepel!(ax, pts; text = ["alpha", "beta", "gamma"])
fig
```

Key attributes: `force`, `force_point`, `force_pull` (anisotropic `(x, y)` tuples),
`only_move` (`:both`/`:x`/`:y`), `max_overlaps` (`Inf` keeps all labels; finite drops
crowded ones), `background` (boxed labels), `segments`/`segmentcolor`/`linewidth`
(connectors).

## How it works

Three layers: **measure** (pixel box sizes via TextMeasure.jl; rich text via Makie),
**solve** (a deterministic force-directed solver in pixel space), **render** (text +
optional boxes + connectors). Output is deterministic — same data, same figure.
```

- [ ] **Step 4: Write `.github/workflows/CI.yml`**

```yaml
name: CI
on:
  push:
    branches: [main]
  pull_request:
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: julia-actions/setup-julia@v2
        with:
          version: '1.11'
      - uses: julia-actions/cache@v2
      - uses: julia-actions/julia-buildpkg@v1
      - uses: julia-actions/julia-runtest@v1
```

The `[sources]` entry in `Project.toml` lets CI resolve TextMeasure from its git URL without registration.

- [ ] **Step 5: Commit**

```bash
git add test/test_integration.jl test/reference/demo.png README.md .github/workflows/CI.yml
git commit -m "Add visual smoke test, README, and CI"
```

---

## Notes / deferred (out of v1 scope)

- **Isolated labels sit on their own marker.** Because the solver skips a label's
  own anchor in point-repulsion (it's the thing the label is attached to via the
  spring), a single uncrowded label stays centered on its data point (offset 0). In
  practice labels in a populated scatter are pushed off markers by neighbouring
  points/labels. A future `nudge`/`point_padding`-from-own-anchor option could force
  an offset for isolated labels if desired.
- **Force model is min-info anisotropic, not min-penetration.** `_aniso_push` pushes
  proportional to overlap on each axis and zeroes a perfectly-aligned axis. This was
  chosen (over standard min-penetration AABB resolution) because min-penetration
  picks the *smaller* overlap axis, which for side-by-side same-row labels is the
  degenerate vertical axis — causing a symmetric runaway. Validated by dry-run.
- **Reactive re-solving** on zoom/pan/resize. The `lift` chain in `plot!` already re-runs the solver when its inputs change; wiring camera/viewport observables to re-project on zoom is a later additive change. Seams are in place.
- **Glyph-coverage fallback** in `measure.jl` (`# TODO(glyph-fallback)`) — narrow mismatched-font/script case.
- **Rounded background corners** (`# TODO(rounded-corners)`) — `cornerradius` is wired but the v1 path draws plain rectangles.
- **Pixel-exact `ReferenceTests.jl`** baseline comparison — v1 ships a render-without-error smoke test plus a manually-inspected `demo.png`.
- **`text_bb`-for-rich-text** is impossible (errors); rich path uses `full_boundingbox` after a render. Replace with TextMeasure once [jowch/TextMeasure.jl#1](https://github.com/jowch/TextMeasure.jl/issues/1) lands.
