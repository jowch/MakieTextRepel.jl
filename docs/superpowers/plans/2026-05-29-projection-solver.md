# ProjectionSolver (side-select → legalize) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the force-loop default solver with a `ProjectionSolver` that performs discrete Imhof side-selection, crossing repair, then Dykstra constraint-projection legalization — guaranteeing zero box overlap on feasible scenes and roughly halving leader length, fully rng-free.

**Architecture:** Three new pure (GeometryBasics-only) layers — `cost.jl` (read-only Q functional), `legalize.jl` (Dykstra cyclic-projection legalizer), `side_select.jl` (greedy discrete slot search) — composed by a thin `ProjectionSolver <: AbstractClusterSolver` in `solvers/projection.jl`. Both surfaces swap their `ForceSolver(params)` construction for `ProjectionSolver(params)`; `ForceSolver` stays in-tree. Reuses v0.2's `voronoi_cells`, `initial_offsets`, `slot_offset`, `repair_crossings!`, `connector_for`, `find_crossings` unchanged.

**Tech Stack:** Julia 1.11, GeometryBasics (`Point2f`/`Vec2f`/`Rect2f`), Makie 0.24 (only at the recipe/annotation seams), Test stdlib, CairoMakie (integration tests only).

**Spec:** `docs/superpowers/specs/2026-05-29-projection-solver-design.md`

---

## File structure

| File | Responsibility | New? |
|------|----------------|------|
| `src/cost.jl` | `label_cost` — pure read-only Q: `(overlaps, mean_leader, crossings)` | NEW |
| `src/legalize.jl` | `dykstra!` + `legalize` — constraint-projection overlap removal | NEW |
| `src/side_select.jl` | `side_select` — greedy discrete Imhof-slot refinement | NEW |
| `src/solvers/projection.jl` | `ProjectionSolver`, `drop_most_overlapped!`, `solve_cluster` | NEW |
| `src/MakieTextRepel.jl` | add four `include`s (no export changes) | modify |
| `src/recipe.jl:97` | `ForceSolver(params)` → `ProjectionSolver(params)` | modify |
| `src/annotation_algorithm.jl` | swap solver; extend `solve_stats` + struct | modify |
| `test/test_cost.jl`, `test_legalize.jl`, `test_side_select.jl`, `test_projection_solver.jl` | unit tests | NEW |
| `test/runtests.jl` | register the four new test files | modify |
| `test/test_integration.jl`, `test/test_solver.jl`, `test/test_annotation_algorithm.jl` | new invariants / rebaseline | modify |
| `Project.toml`, `CHANGELOG.md` | v0.3.0 release prep | modify |

### Conventions used throughout (read before any task)

- **Padded size** of label `i`: `psize[i] = sizes[i] .+ Vec2f(2·box_padding, 2·box_padding)`; padded half-extents `hw = psize.x/2`, `hh = psize.y/2`.
- **Center** of label `i`: `anchors[i] .+ offsets[i]`.
- **Overlap test (inline, fast):** `overlap_push(b1, b2) != Vec2f(0, 0)` (`src/geometry.jl:28`) — used by `side_select`, `drop_most_overlapped!`.
- **Overlap test (legalizer convergence):** penetration `> 0.01` on both axes.
- **Overlap test (Q reporting / over-capacity / drop-stop):** penetration `> 0.5` on both axes.
- All four new files live **inside** `module MakieTextRepel`, so every existing internal (`box_at`, `overlap_push`, `slot_offset`, `IMHOF_ORDER`, `_constrain`, `initial_offsets`, `voronoi_cells`, `repair_crossings!`, `connector_for`, `find_crossings`, `RepelParams`, `Connector`) is in scope **without** import. Tests reach internals via `using MakieTextRepel: <name>` (nothing new is exported).

### Running tests (important — precompilation is slow)

- **Pure-layer tests** (Tasks 1–4) need only MakieTextRepel + GeometryBasics + Test (all available in `--project=.`); run a single file directly — fast, no CairoMakie:
  ```bash
  julia --project=. test/test_legalize.jl
  ```
- **Integration / full-suite runs** (Tasks 5–8) need CairoMakie (a test-only extra). Run the whole suite once, tee to an agent-scoped log under the gitignored `test/output/`, then grep — do **not** re-run per step:
  ```bash
  LOG="test/output/test-projection.log"
  julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
  grep -E "Test Summary|Fail|Error|Pass" "$LOG"
  ```

---

## Task 1: `cost.jl` — read-only Q functional

**Files:**
- Create: `src/cost.jl`
- Modify: `src/MakieTextRepel.jl` (add include)
- Test: `test/test_cost.jl`

- [ ] **Step 1: Write the failing test**

Create `test/test_cost.jl`:

```julia
using MakieTextRepel
using MakieTextRepel: label_cost
using GeometryBasics
using Test

@testset "label_cost" begin
    bounds = Rect2f(0, 0, 200, 200)
    bp = 0.0   # zero padding so geometry is easy to reason about

    # Two 20×20 labels. anchors at (50,50) and (50,50); offsets put centers
    # 10px apart in x → boxes (half-width 10 each) penetrate 10px on x, fully overlap y.
    anchors = [Point2f(50, 50), Point2f(50, 50)]
    sizes   = [Vec2f(20, 20), Vec2f(20, 20)]
    offs_ov = [Vec2f(0, 0), Vec2f(10, 0)]
    q = label_cost(anchors, sizes; offsets = offs_ov, bounds = bounds,
                   box_padding = bp, min_segment_length = 2.0)
    @test q.overlaps == 1                       # 10px > 0.5 both axes
    @test q.mean_leader ≈ Float32((0 + 10) / 2) # ‖(0,0)‖=0, ‖(10,0)‖=10

    # Separate them 40px in x → centers 40 apart, half-widths sum 20 → no overlap.
    offs_sep = [Vec2f(0, 0), Vec2f(40, 0)]
    q2 = label_cost(anchors, sizes; offsets = offs_sep, bounds = bounds,
                    box_padding = bp, min_segment_length = 2.0)
    @test q2.overlaps == 0

    # Sub-0.5px touch is NOT counted as an overlap (the Q threshold split).
    offs_near = [Vec2f(0, 0), Vec2f(19.7, 0)]  # penetration 20-19.7 = 0.3 < 0.5 → not counted
    @test label_cost(anchors, sizes; offsets = offs_near, bounds = bounds,
                     box_padding = bp, min_segment_length = 2.0).overlaps == 0

    # Dropped labels are excluded from overlaps and mean_leader.
    dropped = BitVector([false, true])
    q3 = label_cost(anchors, sizes; offsets = offs_ov, bounds = bounds, dropped = dropped,
                    box_padding = bp, min_segment_length = 2.0)
    @test q3.overlaps == 0                       # label 2 dropped → pair excluded
    @test q3.mean_leader ≈ 0f0                   # only label 1 active, ‖(0,0)‖ = 0

    # crossings: cross-check against a hand-built crossing fixture (two leaders that cross).
    a = [Point2f(0, 0), Point2f(10, 0)]
    s = [Vec2f(4, 4), Vec2f(4, 4)]
    o = [Vec2f(10, 4), Vec2f(-10, 4)]            # label1 goes up-right, label2 up-left → cross
    qc = label_cost(a, s; offsets = o, bounds = Rect2f(-50, -50, 100, 100),
                    box_padding = bp, min_segment_length = 0.5)
    @test qc.crossings == 1
end
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `julia --project=. test/test_cost.jl`
Expected: FAIL — `UndefVarError: label_cost not defined` (or `LoadError` wrapping it).

- [ ] **Step 3: Create `src/cost.jl`**

```julia
# cost.jl — read-only placement quality functional (Q). Pure, GeometryBasics-only.

"""
    label_cost(anchors, sizes; offsets, bounds, dropped=nothing,
               box_padding, point_padding=0.0, min_segment_length)
        -> (; overlaps::Int, mean_leader::Float32, crossings::Int)

Read-only quality measure of a placement. Never mutates, never feeds back into
placement. Three independent components (reported separately, never collapsed):

- `overlaps`  — count of label pairs whose padded boxes penetrate by > 0.5 px on
  both axes (the visual-overlap threshold; sub-0.5 px touches are ignored).
- `mean_leader` — mean ‖offset‖ over non-dropped labels (0 if none).
- `crossings` — crossing leader pairs, via `connector_for` + `find_crossings`.

Labels flagged in `dropped` are excluded from `overlaps` and `mean_leader`
(they are render-suppressed). `box_padding`/`point_padding`/`min_segment_length`
must match the recipe values so the crossing count agrees with what renders.
"""
function label_cost(anchors::Vector{Point2f}, sizes::Vector{Vec2f};
                    offsets::Vector{Vec2f}, bounds::Rect2f,
                    dropped::Union{Nothing,BitVector} = nothing,
                    box_padding::Real, point_padding::Real = 0.0,
                    min_segment_length::Real)
    n = length(anchors)
    isdrp(i) = dropped !== nothing && dropped[i]
    hw = [Float64(sizes[i][1] + 2box_padding) / 2 for i in 1:n]
    hh = [Float64(sizes[i][2] + 2box_padding) / 2 for i in 1:n]
    cx = [Float64(anchors[i][1] + offsets[i][1]) for i in 1:n]
    cy = [Float64(anchors[i][2] + offsets[i][2]) for i in 1:n]

    overlaps = 0
    for i in 1:n, j in (i+1):n
        (isdrp(i) || isdrp(j)) && continue
        ox = (hw[i] + hw[j]) - abs(cx[i] - cx[j])
        oy = (hh[i] + hh[j]) - abs(cy[i] - cy[j])
        (ox > 0.5 && oy > 0.5) && (overlaps += 1)
    end

    active = [i for i in 1:n if !isdrp(i)]
    mean_leader = isempty(active) ? 0f0 :
        Float32(sum(sqrt(Float64(offsets[i][1])^2 + Float64(offsets[i][2])^2)
                    for i in active) / length(active))

    params = RepelParams(; box_padding = box_padding, point_padding = point_padding)
    connectors = [connector_for(anchors[i], offsets[i], sizes[i], isdrp(i),
                                params, min_segment_length) for i in 1:n]
    crossings = length(find_crossings(connectors))

    return (; overlaps = overlaps, mean_leader = mean_leader, crossings = crossings)
end
```

- [ ] **Step 4: Wire the include**

In `src/MakieTextRepel.jl`, add `include("cost.jl")` immediately after `include("crossings.jl")` (line 17), so `connector_for`/`find_crossings` are already defined:

```julia
    include("crossings.jl")
    include("cost.jl")
    include("measure.jl")
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `julia --project=. test/test_cost.jl`
Expected: PASS — `Test Summary: | Pass  Total` with all `label_cost` assertions green.

- [ ] **Step 6: Commit**

```bash
git add src/cost.jl src/MakieTextRepel.jl test/test_cost.jl
git commit -m "feat: add label_cost read-only Q functional (cost.jl)"
```

---

## Task 2: `legalize.jl` — Dykstra constraint-projection legalizer

**Files:**
- Create: `src/legalize.jl`
- Modify: `src/MakieTextRepel.jl` (add include)
- Test: `test/test_legalize.jl`

- [ ] **Step 1: Write the failing test**

Create `test/test_legalize.jl`:

```julia
using MakieTextRepel
using MakieTextRepel: legalize, dykstra!
using GeometryBasics
using Test

# helper: count >0.5px overlapping pairs among movable+fixed boxes
function novl(anchors, offsets, psizes)
    n = length(anchors); c = 0
    for i in 1:n, j in (i+1):n
        cx = (anchors[i][1]+offsets[i][1]) - (anchors[j][1]+offsets[j][1])
        cy = (anchors[i][2]+offsets[i][2]) - (anchors[j][2]+offsets[j][2])
        ox = (psizes[i][1]+psizes[j][1])/2 - abs(cx)
        oy = (psizes[i][2]+psizes[j][2])/2 - abs(cy)
        (ox > 0.5 && oy > 0.5) && (c += 1)
    end
    c
end

@testset "dykstra! single constraint splits the move" begin
    pos = Float64[0.0, 1.0]                 # need pos[2]-pos[1] >= 4
    dykstra!(pos, [(1, 2, 4.0)], Bool[true, true])
    @test pos[2] - pos[1] ≈ 4.0 atol=1e-2
    @test pos[1] ≈ -1.5 atol=1e-2           # moved symmetrically: gap 3 → split 1.5 each
    @test pos[2] ≈ 2.5  atol=1e-2
end

@testset "dykstra! fixed endpoint absorbs nothing" begin
    pos = Float64[0.0, 1.0]
    dykstra!(pos, [(1, 2, 4.0)], Bool[false, true])  # lo fixed → only hi moves
    @test pos[1] ≈ 0.0 atol=1e-9
    @test pos[2] ≈ 4.0 atol=1e-2
end

@testset "legalize reaches zero overlap on a feasible cluster" begin
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(200, 200) for _ in 1:6]
    psizes  = [Vec2f(40, 20) for _ in 1:6]
    offsets = [Vec2f(3i, 2i) for i in 1:6]            # heavily overlapping fan
    fixed = falses(6)
    r = legalize(anchors, offsets, psizes, bounds; fixed = fixed)
    @test r.residual ≤ 0.5f0
    @test novl(anchors, r.offsets, psizes) == 0
end

@testset "legalize signals over-capacity when infeasible" begin
    # 9 boxes of 60×60 cannot fit overlap-free in 100×100 bounds.
    bounds = Rect2f(0, 0, 100, 100)
    anchors = [Point2f(50, 50) for _ in 1:9]
    psizes  = [Vec2f(60, 60) for _ in 1:9]
    offsets = [Vec2f(0, 0) for _ in 1:9]
    r = legalize(anchors, offsets, psizes, bounds; fixed = falses(9), rounds = 200)
    @test r.residual > 0.5f0                          # cannot clear → over-capacity
end

@testset "legalize holds fixed nodes still" begin
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(200, 200), Point2f(200, 200)]
    psizes  = [Vec2f(40, 40), Vec2f(40, 40)]
    offsets = [Vec2f(0, 0), Vec2f(5, 0)]              # overlapping
    fixed = BitVector([true, false])                  # label 1 pinned
    r = legalize(anchors, offsets, psizes, bounds; fixed = fixed)
    @test r.offsets[1] == Vec2f(0, 0)                 # bit-identical: fixed node never moved
    @test novl(anchors, r.offsets, psizes) == 0       # label 2 absorbed the separation
end

@testset "legalize only_move=:x leaves y untouched" begin
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(200, 200), Point2f(200, 200)]
    psizes  = [Vec2f(40, 40), Vec2f(40, 40)]
    offsets = [Vec2f(0, 0), Vec2f(5, 3)]
    r = legalize(anchors, offsets, psizes, bounds; fixed = falses(2), only_move = :x)
    @test (anchors[1][2] + r.offsets[1][2]) ≈ 200.0 atol=1e-4   # y centers unchanged
    @test (anchors[2][2] + r.offsets[2][2]) ≈ 203.0 atol=1e-4
end

@testset "legalize is a no-op (bit-identical) on a separated layout" begin
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(100, 100), Point2f(300, 300)]
    psizes  = [Vec2f(40, 40), Vec2f(40, 40)]
    offsets = [Vec2f(0, 0), Vec2f(0, 0)]
    r = legalize(anchors, offsets, psizes, bounds; fixed = falses(2))
    @test r.rounds_used == 0
    @test r.offsets[1] === offsets[1]                 # exact same Vec2f preserved
    @test r.offsets[2] === offsets[2]
end

@testset "legalize is deterministic" begin
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(200, 200) for _ in 1:8]
    psizes  = [Vec2f(30, 18) for _ in 1:8]
    offsets = [Vec2f(2i, i) for i in 1:8]
    r1 = legalize(anchors, offsets, psizes, bounds; fixed = falses(8))
    r2 = legalize(anchors, offsets, psizes, bounds; fixed = falses(8))
    @test r1.offsets == r2.offsets
end
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `julia --project=. test/test_legalize.jl`
Expected: FAIL — `UndefVarError: legalize not defined`.

- [ ] **Step 3: Create `src/legalize.jl`**

```julia
# legalize.jl — Dykstra constraint-projection node-overlap removal. Pure, GeometryBasics-only.
#
# Per round: collect currently-overlapping pairs, assign each to its cheaper
# (smaller-penetration) axis as a 1-D separation constraint, then project onto
# those constraints with Dykstra's cyclic algorithm (the minimum-displacement
# point satisfying them — the QP VPSC solves), clamp to bounds, repeat. Fixed
# nodes contribute their extents but never move. Caps at `rounds`; a positive
# returned `residual` means the scene could not be cleared in-bounds (over-capacity).

"""
Dykstra cyclic projection onto separation constraints `pos[hi] - pos[lo] >= gap`.
`movable[k]` false pins node k (its endpoint absorbs none of a violation). A
constraint between two fixed nodes is left unsatisfied (residual). Mutates `pos`.
"""
function dykstra!(pos::Vector{Float64}, cons::Vector{Tuple{Int,Int,Float64}},
                  movable::Vector{Bool}; iters::Int = 5000, tol::Float64 = 1e-3)
    m = length(cons); m == 0 && return pos
    Ilo = zeros(m); Ihi = zeros(m)          # Dykstra per-constraint correction memory
    for _ in 1:iters
        changed = 0.0
        for k in 1:m
            lo, hi, gap = cons[k]
            ylo = pos[lo] + Ilo[k]; yhi = pos[hi] + Ihi[k]
            viol = gap - (yhi - ylo)
            if viol > 0
                ml = movable[lo]; mh = movable[hi]
                if ml && mh
                    xlo = ylo - viol/2; xhi = yhi + viol/2
                elseif ml
                    xlo = ylo - viol;   xhi = yhi
                elseif mh
                    xlo = ylo;          xhi = yhi + viol
                else
                    xlo = ylo;          xhi = yhi      # both fixed: unsatisfiable
                end
            else
                xlo = ylo; xhi = yhi
            end
            Ilo[k] = ylo - xlo; Ihi[k] = yhi - xhi
            changed = max(changed, abs(pos[lo] - xlo), abs(pos[hi] - xhi))
            pos[lo] = xlo; pos[hi] = xhi
        end
        changed < tol && break
    end
    return pos
end

"""
    legalize(anchors, offsets, psizes, bounds; fixed, only_move=:both, rounds=400)
        -> (; offsets::Vector{Vec2f}, residual::Float32, rounds_used::Int)

Move boxes the minimum distance needed so no two padded boxes overlap. `psizes`
are padded sizes; box i is centered at `anchors[i] + offsets[i]` with half-extents
`psizes[i]/2`. `fixed[i]` pins box i (contributes extents, never moves) — obstacle
pseudo-nodes are passed as fixed entries by the caller. `only_move` restricts
separation to one axis. `residual` is the max remaining >0.01px penetration after
the round cap; >0.5 means over-capacity. Offsets of un-moved labels are returned
bit-identical to the input.
"""
function legalize(anchors::Vector{Point2f}, offsets::Vector{Vec2f},
                  psizes::Vector{Vec2f}, bounds::Rect2f;
                  fixed::BitVector, only_move::Symbol = :both, rounds::Int = 400)
    n = length(anchors)
    x  = [Float64(anchors[i][1] + offsets[i][1]) for i in 1:n]
    y  = [Float64(anchors[i][2] + offsets[i][2]) for i in 1:n]
    x0 = copy(x); y0 = copy(y)              # originals, for bit-identity preservation
    hw = [Float64(psizes[i][1]) / 2 for i in 1:n]
    hh = [Float64(psizes[i][2]) / 2 for i in 1:n]
    movable = [!fixed[i] for i in 1:n]
    blo = bounds.origin; bw = bounds.widths
    xlo_b = Float64(blo[1]); ylo_b = Float64(blo[2])
    xhi_b = Float64(blo[1] + bw[1]); yhi_b = Float64(blo[2] + bw[2])

    rounds_used = rounds
    for r in 1:rounds
        xcons = Tuple{Int,Int,Float64}[]
        ycons = Tuple{Int,Int,Float64}[]
        for i in 1:n, j in (i+1):n
            (!movable[i] && !movable[j]) && continue        # two fixed nodes: skip
            ox = (hw[i] + hw[j]) - abs(x[i] - x[j])
            oy = (hh[i] + hh[j]) - abs(y[i] - y[j])
            if ox > 0.01 && oy > 0.01
                usex = only_move === :x ? true :
                       only_move === :y ? false : (ox <= oy)
                if usex
                    lo, hi = x[i] <= x[j] ? (i, j) : (j, i)
                    push!(xcons, (lo, hi, hw[i] + hw[j]))
                else
                    lo, hi = y[i] <= y[j] ? (i, j) : (j, i)
                    push!(ycons, (lo, hi, hh[i] + hh[j]))
                end
            end
        end
        if isempty(xcons) && isempty(ycons)
            rounds_used = r - 1
            break
        end
        dykstra!(x, xcons, movable)
        dykstra!(y, ycons, movable)
        for i in 1:n
            movable[i] || continue
            x[i] = clamp(x[i], xlo_b + hw[i], xhi_b - hw[i])
            y[i] = clamp(y[i], ylo_b + hh[i], yhi_b - hh[i])
        end
    end

    # final residual (post-clamp): max >0.01px penetration still present
    finalpen = 0.0
    for i in 1:n, j in (i+1):n
        (!movable[i] && !movable[j]) && continue
        ox = (hw[i] + hw[j]) - abs(x[i] - x[j])
        oy = (hh[i] + hh[j]) - abs(y[i] - y[j])
        (ox > 0.01 && oy > 0.01) && (finalpen = max(finalpen, min(ox, oy)))
    end

    new_offsets = Vector{Vec2f}(undef, n)
    for i in 1:n
        if x[i] == x0[i] && y[i] == y0[i]
            new_offsets[i] = offsets[i]                     # unmoved → preserve exact value
        else
            new_offsets[i] = Vec2f(x[i] - anchors[i][1], y[i] - anchors[i][2])
        end
    end
    return (; offsets = new_offsets, residual = Float32(finalpen), rounds_used = rounds_used)
end
```

- [ ] **Step 4: Wire the include**

In `src/MakieTextRepel.jl`, add `include("legalize.jl")` immediately after the `include("cost.jl")` line added in Task 1:

```julia
    include("cost.jl")
    include("legalize.jl")
    include("measure.jl")
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `julia --project=. test/test_legalize.jl`
Expected: PASS — all eight `@testset`s green (zero-overlap, over-capacity, fixed-node, only_move, no-op bit-identity, determinism).

- [ ] **Step 6: Commit**

```bash
git add src/legalize.jl src/MakieTextRepel.jl test/test_legalize.jl
git commit -m "feat: add Dykstra constraint-projection legalizer (legalize.jl)"
```

---

## Task 3: `side_select.jl` — greedy discrete Imhof-slot refinement

**Files:**
- Create: `src/side_select.jl`
- Modify: `src/MakieTextRepel.jl` (add include)
- Test: `test/test_side_select.jl`

- [ ] **Step 1: Write the failing test**

Create `test/test_side_select.jl`:

```julia
using MakieTextRepel
using MakieTextRepel: side_select, slot_offset, box_at, overlap_push, IMHOF_ORDER, RepelParams
using GeometryBasics
using Test

psz(sizes, bp) = [s .+ Vec2f(2bp, 2bp) for s in sizes]
anyov(anchors, sel, psizes) = any(
    overlap_push(box_at(anchors[i], sel[i], psizes[i]),
                 box_at(anchors[j], sel[j], psizes[j])) != Vec2f(0, 0)
    for i in 1:length(anchors) for j in (i+1):length(anchors))

@testset "side_select keeps seed slot when there is no conflict" begin
    p = RepelParams()
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(100, 100), Point2f(300, 300)]   # far apart, no conflict
    sizes   = [Vec2f(30, 16), Vec2f(30, 16)]
    ps = psz(sizes, p.box_padding)
    seed = [slot_offset(:TR, sizes[i], p.point_padding) for i in 1:2]
    sel = side_select(anchors, sizes, ps, bounds, seed, p)
    @test sel[1] ≈ slot_offset(:TR, sizes[1], p.point_padding)
    @test sel[2] ≈ slot_offset(:TR, sizes[2], p.point_padding)
end

@testset "side_select resolves a head-on conflict to opposite sides" begin
    p = RepelParams()
    bounds = Rect2f(0, 0, 400, 400)
    # Two anchors close in x; if both pick TR they overlap. Greedy should split them.
    anchors = [Point2f(195, 200), Point2f(205, 200)]
    sizes   = [Vec2f(60, 16), Vec2f(60, 16)]
    ps = psz(sizes, p.box_padding)
    seed = [slot_offset(:TR, sizes[i], p.point_padding) for i in 1:2]
    sel = side_select(anchors, sizes, ps, bounds, seed, p)
    @test !anyov(anchors, sel, ps)                      # overlap term dominates → separated
end

@testset "side_select fixes pinned labels and avoids obstacles" begin
    p = RepelParams()
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(200, 200), Point2f(260, 200)]
    sizes   = [Vec2f(40, 16), Vec2f(40, 16)]
    ps = psz(sizes, p.box_padding)
    seed = [slot_offset(:TR, sizes[i], p.point_padding) for i in 1:2]
    pin = BitVector([true, false])
    pinned = [Vec2f(0, -50), Vec2f(0, 0)]               # label 1 pinned below
    sel = side_select(anchors, sizes, ps, bounds, seed, p;
                      pin_mask = pin, pinned_offsets = pinned)
    @test sel[1] == Vec2f(0, -50)                       # pinned: untouched
end

@testset "side_select only_move=:x restricts to horizontal slots" begin
    p = RepelParams(; only_move = :x)
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(100, 100)]
    sizes   = [Vec2f(30, 16)]
    ps = psz(sizes, p.box_padding)
    seed = [slot_offset(:TR, sizes[1], p.point_padding)]
    sel = side_select(anchors, sizes, ps, bounds, seed, p)
    @test sel[1][2] ≈ 0f0                                # y-component locked to 0
end

@testset "side_select is deterministic" begin
    p = RepelParams()
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(195, 200), Point2f(205, 200), Point2f(200, 215)]
    sizes   = [Vec2f(50, 16) for _ in 1:3]
    ps = psz(sizes, p.box_padding)
    seed = [slot_offset(:TR, sizes[i], p.point_padding) for i in 1:3]
    s1 = side_select(anchors, sizes, ps, bounds, seed, p)
    s2 = side_select(anchors, sizes, ps, bounds, seed, p)
    @test s1 == s2
end
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `julia --project=. test/test_side_select.jl`
Expected: FAIL — `UndefVarError: side_select not defined`.

- [ ] **Step 3: Create `src/side_select.jl`**

```julia
# side_select.jl — greedy discrete Imhof-slot refinement. Pure, GeometryBasics-only.
#
# Each label's candidate offsets are its in-bounds Imhof slots (constrained by
# only_move). Seeded from the Voronoi-informed init (nearest candidate to the
# seed), then refined by index-ordered greedy sweeps minimizing
#   cost(slot) = overlap_count·overlap_weight + ‖offset‖
# (overlap-avoidance dominates; leader length is the tiebreak). Pinned labels are
# fixed at their pinned offset; obstacles count as fixed boxes. Deterministic.

"""
    side_select(anchors, sizes, psizes, bounds, seed, params;
                pin_mask=nothing, pinned_offsets=Vec2f[], obstacles=Rect2f[],
                overlap_weight=1000.0, passes=6) -> Vector{Vec2f}

Pick, per label, the Imhof slot minimizing overlap then leader length. `psizes`
are padded sizes. `seed[i]` is the Voronoi-informed initial offset used to choose
the starting slot. Returns the chosen offset per label.
"""
function side_select(anchors::Vector{Point2f}, sizes::Vector{Vec2f},
                     psizes::Vector{Vec2f}, bounds::Rect2f,
                     seed::Vector{Vec2f}, params;
                     pin_mask::Union{Nothing,BitVector} = nothing,
                     pinned_offsets::Vector{Vec2f}       = Vec2f[],
                     obstacles::Vector{Rect2f}           = Rect2f[],
                     overlap_weight::Float64 = 1000.0, passes::Int = 6)
    n = length(anchors)
    p = params.point_padding
    blo = bounds.origin; bw = bounds.widths
    inb(b) = b.origin[1] >= blo[1] - 1e-3 && b.origin[2] >= blo[2] - 1e-3 &&
             b.origin[1] + b.widths[1] <= blo[1] + bw[1] + 1e-3 &&
             b.origin[2] + b.widths[2] <= blo[2] + bw[2] + 1e-3

    # candidate offsets per label: in-bounds, only_move-constrained Imhof slots
    # (keep all eight if none fit, so the legalizer can rescue it later)
    cands = Vector{Vector{Vec2f}}(undef, n)
    for i in 1:n
        cs = Vec2f[]
        for s in IMHOF_ORDER
            o = _constrain(slot_offset(s, sizes[i], p), params.only_move)
            inb(box_at(anchors[i], o, psizes[i])) && push!(cs, o)
        end
        isempty(cs) && (cs = [_constrain(slot_offset(s, sizes[i], p), params.only_move)
                              for s in IMHOF_ORDER])
        cands[i] = cs
    end

    isfixed(i) = pin_mask !== nothing && pin_mask[i]

    # initial selection: pinned → fixed offset; else nearest candidate to seed
    sel = Vector{Vec2f}(undef, n)
    for i in 1:n
        if isfixed(i)
            sel[i] = pinned_offsets[i]
        else
            best = cands[i][1]; bestd = Inf
            for o in cands[i]
                d = (Float64(o[1]) - seed[i][1])^2 + (Float64(o[2]) - seed[i][2])^2
                if d < bestd; bestd = d; best = o; end
            end
            sel[i] = best
        end
    end

    # Global cost of an arrangement (overlap pairs · weight + total leader length).
    # Greedy best-response is NOT globally monotone and can 2-cycle, so we keep the
    # best arrangement seen across passes rather than trusting the last pass.
    function global_cost(s)
        tot = 0.0
        for i in 1:n
            b = box_at(anchors[i], s[i], psizes[i])
            for j in (i+1):n
                (overlap_push(b, box_at(anchors[j], s[j], psizes[j])) != Vec2f(0, 0)) && (tot += overlap_weight)
            end
            for ob in obstacles
                (overlap_push(b, ob) != Vec2f(0, 0)) && (tot += overlap_weight)
            end
            tot += sqrt(Float64(s[i][1])^2 + Float64(s[i][2])^2)
        end
        return tot
    end

    best_sel = copy(sel); best_cost = global_cost(sel)
    for _ in 1:passes
        changed = false
        for i in 1:n
            isfixed(i) && continue
            besto = sel[i]; bestc = Inf
            for o in cands[i]
                b = box_at(anchors[i], o, psizes[i])
                ov = 0
                for j in 1:n
                    j == i && continue
                    (overlap_push(b, box_at(anchors[j], sel[j], psizes[j])) != Vec2f(0, 0)) && (ov += 1)
                end
                for ob in obstacles
                    (overlap_push(b, ob) != Vec2f(0, 0)) && (ov += 1)
                end
                c = ov * overlap_weight + sqrt(Float64(o[1])^2 + Float64(o[2])^2)
                if c < bestc; bestc = c; besto = o; end
            end
            (besto != sel[i]) && (changed = true)
            sel[i] = besto
        end
        gc = global_cost(sel)
        if gc < best_cost; best_cost = gc; best_sel = copy(sel); end
        changed || break
    end
    return best_sel
end
```

- [ ] **Step 4: Wire the include**

In `src/MakieTextRepel.jl`, add `include("side_select.jl")` immediately after `include("legalize.jl")`:

```julia
    include("legalize.jl")
    include("side_select.jl")
    include("measure.jl")
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `julia --project=. test/test_side_select.jl`
Expected: PASS — all five `@testset`s green.

- [ ] **Step 6: Commit**

```bash
git add src/side_select.jl src/MakieTextRepel.jl test/test_side_select.jl
git commit -m "feat: add greedy discrete Imhof side-selection (side_select.jl)"
```

---

## Task 4: `solvers/projection.jl` — `ProjectionSolver`

**Files:**
- Create: `src/solvers/projection.jl`
- Modify: `src/MakieTextRepel.jl` (add include)
- Test: `test/test_projection_solver.jl`

- [ ] **Step 1: Write the failing test**

Create `test/test_projection_solver.jl`:

```julia
using MakieTextRepel
using MakieTextRepel: ProjectionSolver, ForceSolver, solve_cluster, drop_most_overlapped!,
                      box_at, overlap_push, label_cost
using GeometryBasics
using Test

# deterministic stand-in for randn so the suite is rng-free and reproducible.
# Defined first because @testset bodies execute top-to-bottom as the file loads.
randn_stub(i) = sin(Float64(i) * 12.9898) * 4.0

psz(sizes, bp) = [s .+ Vec2f(2bp, 2bp) for s in sizes]
function novl(anchors, offsets, sizes, bp; dropped = nothing)
    ps = psz(sizes, bp); n = length(anchors); c = 0
    for i in 1:n, j in (i+1):n
        (dropped !== nothing && (dropped[i] || dropped[j])) && continue
        ox = (ps[i][1]+ps[j][1])/2 - abs((anchors[i][1]+offsets[i][1]) - (anchors[j][1]+offsets[j][1]))
        oy = (ps[i][2]+ps[j][2])/2 - abs((anchors[i][2]+offsets[i][2]) - (anchors[j][2]+offsets[j][2]))
        (ox > 0.5 && oy > 0.5) && (c += 1)
    end
    c
end
mean_leader(offsets, dropped) = begin
    act = [i for i in eachindex(offsets) if !dropped[i]]
    isempty(act) ? 0.0 : sum(sqrt(Float64(offsets[i][1])^2 + offsets[i][2]^2) for i in act)/length(act)
end

@testset "ProjectionSolver: zero overlap per scene; shorter leader in aggregate" begin
    # Per-scene leader length is NOT a theorem (a sparse scene's force_pull can hug the
    # anchor closer than a discrete Imhof slot). §7e validated the win in AGGREGATE
    # (~2× shorter across knot/collinear/sparse). So assert zero-overlap per scene
    # (the hard guarantee) and the leader win as an aggregate with a real margin.
    p = RepelParams()
    fixtures = [
        ("knot",      Rect2f(0, 0, 380, 340),
         [Point2f(90 + 8randn_stub(i), 250 + 8randn_stub(i+1)) for i in 1:12], Vec2f(40, 15)),
        ("collinear", Rect2f(0, 0, 380, 340),
         [Point2f(40 + 30i, 180) for i in 0:9], Vec2f(40, 15)),
        ("scatter",   Rect2f(0, 0, 380, 340),
         [Point2f(190 + 60randn_stub(i), 170 + 50randn_stub(i+2)) for i in 1:18], Vec2f(35, 15)),
    ]
    proj_total = 0.0; force_total = 0.0
    for (name, bounds, anchors, sz) in fixtures
        sizes = [sz for _ in 1:length(anchors)]
        rp = solve_cluster(ProjectionSolver(p), anchors, sizes, bounds)
        rf = solve_cluster(ForceSolver(p), anchors, sizes, bounds)
        @test novl(anchors, rp.offsets, sizes, p.box_padding; dropped = rp.dropped) == 0  # hard, per scene
        proj_total  += mean_leader(rp.offsets, rp.dropped)
        force_total += mean_leader(rf.offsets, rf.dropped)
    end
    @test proj_total ≤ 0.9 * force_total      # aggregate leader win, generous margin
end

@testset "ProjectionSolver: over-capacity drops, survivors are overlap-free" begin
    p = RepelParams()
    bounds = Rect2f(0, 0, 80, 80)                       # tiny bounds
    anchors = [Point2f(40, 40) for _ in 1:8]
    sizes   = [Vec2f(40, 30) for _ in 1:8]              # can't all fit
    r = solve_cluster(ProjectionSolver(p), anchors, sizes, bounds)
    @test count(r.dropped) > 0
    @test novl(anchors, r.offsets, sizes, p.box_padding; dropped = r.dropped) == 0
end

@testset "ProjectionSolver: warm-start (init_state) legalizes without re-siding" begin
    p = RepelParams()
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(200, 200), Point2f(200, 200)]
    sizes   = [Vec2f(40, 40), Vec2f(40, 40)]
    # pre-separated init → must come back essentially unchanged
    init = [Vec2f(-60, 0), Vec2f(60, 0)]
    r = solve_cluster(ProjectionSolver(p), anchors, sizes, bounds; init_state = init)
    @test r.offsets[1] ≈ init[1] atol=0.5
    @test r.offsets[2] ≈ init[2] atol=0.5
    # overlapping init → separated, but offsets stay near the supplied init (no slot search)
    init2 = [Vec2f(0, 0), Vec2f(6, 0)]
    r2 = solve_cluster(ProjectionSolver(p), anchors, sizes, bounds; init_state = init2)
    @test novl(anchors, r2.offsets, sizes, p.box_padding) == 0
end

@testset "ProjectionSolver: pinned offsets are preserved bit-identically" begin
    p = RepelParams()
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(150, 200), Point2f(250, 200)]
    sizes   = [Vec2f(40, 20), Vec2f(40, 20)]
    pin = BitVector([true, false])
    pinned = [Vec2f(0, -70), Vec2f(0, 0)]
    r = solve_cluster(ProjectionSolver(p), anchors, sizes, bounds;
                      pin_mask = pin, pinned_offsets = pinned)
    @test r.offsets[1] == Vec2f(0, -70)
end

@testset "ProjectionSolver: deterministic" begin
    p = RepelParams()
    bounds = Rect2f(0, 0, 380, 340)
    anchors = [Point2f(100 + 5i, 150 + 3i) for i in 1:10]
    sizes   = [Vec2f(35, 15) for _ in 1:10]
    r1 = solve_cluster(ProjectionSolver(p), anchors, sizes, bounds)
    r2 = solve_cluster(ProjectionSolver(p), anchors, sizes, bounds)
    @test r1.offsets == r2.offsets
    @test r1.dropped == r2.dropped
end

@testset "drop_most_overlapped! picks the busiest box, ties by highest index" begin
    p = RepelParams()
    anchors = [Point2f(0, 0), Point2f(0, 0), Point2f(0, 0)]
    sizes   = [Vec2f(20, 20) for _ in 1:3]
    ps = psz(sizes, p.box_padding)
    offsets = [Vec2f(0, 0), Vec2f(0, 0), Vec2f(0, 0)]   # all coincident → all overlap all
    dropped = falses(3)
    idx = drop_most_overlapped!(dropped, anchors, offsets, ps, nothing)
    @test idx == 3                                       # tie (each overlaps 2) → highest index
    @test dropped[3]
end
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `julia --project=. test/test_projection_solver.jl`
Expected: FAIL — `UndefVarError: ProjectionSolver not defined`.

- [ ] **Step 3: Create `src/solvers/projection.jl`**

```julia
# solvers/projection.jl — ProjectionSolver: side-select → repair → legalize, with
# geometric over-capacity dropping and read-only Q diagnostics. Composes the pure
# layers; the only AbstractClusterSolver that touches solver-internal types here.

"""
`ProjectionSolver` carries `RepelParams` and a `stats` Ref holding the last solve's
Q diagnostics `(overlaps, mean_leader, crossings, iter, residual, dropped)`.
"""
struct ProjectionSolver <: AbstractClusterSolver
    params::RepelParams
    stats::Base.RefValue{NamedTuple}
end

ProjectionSolver(params::RepelParams) =
    ProjectionSolver(params, Ref{NamedTuple}((; overlaps = 0, mean_leader = 0f0,
                                                crossings = 0, iter = 0,
                                                residual = 0f0, dropped = 0)))

"""
Mark the still-active, non-pinned label whose padded box overlaps the most other
active boxes (ties → highest index) as dropped. Returns the dropped index (0 if
none eligible). Deterministic.
"""
function drop_most_overlapped!(dropped::BitVector, anchors::Vector{Point2f},
                               offsets::Vector{Vec2f}, psizes::Vector{Vec2f},
                               pin_mask::Union{Nothing,BitVector})
    n = length(offsets)
    bestidx = 0; bestov = -1
    for i in 1:n
        dropped[i] && continue
        (pin_mask !== nothing && pin_mask[i]) && continue
        bi = box_at(anchors[i], offsets[i], psizes[i])
        ov = 0
        for j in 1:n
            (j == i || dropped[j]) && continue
            (overlap_push(bi, box_at(anchors[j], offsets[j], psizes[j])) != Vec2f(0, 0)) && (ov += 1)
        end
        if ov > bestov || (ov == bestov && i > bestidx)   # ties → highest index
            bestov = ov; bestidx = i
        end
    end
    bestidx > 0 && (dropped[bestidx] = true)
    return bestidx
end

function solve_cluster(s::ProjectionSolver, anchors::Vector{Point2f}, sizes::Vector{Vec2f},
                       bounds::Rect2f;
                       init_state::Union{Nothing,Vector{Vec2f}} = nothing,
                       pin_mask::Union{Nothing,BitVector}        = nothing,
                       pinned_offsets::Vector{Vec2f}             = Vec2f[],
                       obstacles::Vector{Rect2f}                 = Rect2f[])
    p = RepelParams(s.params; bounds = bounds)
    n = length(anchors)
    pad = Float32(p.box_padding)
    psizes = [sizes[i] .+ 2pad for i in 1:n]

    if init_state === nothing       # FRESH: seed → side-select → crossing repair
        seed = initial_offsets(anchors, sizes, voronoi_cells(anchors, bounds), p;
                               pin_mask = pin_mask, pinned_offsets = pinned_offsets)
        offsets = side_select(anchors, sizes, psizes, bounds, seed, p;
                              pin_mask = pin_mask, pinned_offsets = pinned_offsets,
                              obstacles = obstacles)
        repair_crossings!(offsets, anchors, sizes, falses(n), p;
                          min_len = p.min_segment_length, pin_mask = pin_mask)
    else                            # RELAX / warm-start: legalize the given layout only.
        # Mirror solve_repel's init_state contract: constrain to only_move, and hold
        # pinned labels at their fixed offset (the caller's pinned_offsets), not the
        # warm value. Without this, pinned labels would be legalized away from their pin.
        offsets = [_constrain(o, p.only_move) for o in init_state]
        if pin_mask !== nothing
            for i in 1:n
                pin_mask[i] && (offsets[i] = pinned_offsets[i])
            end
        end
    end

    dropped = falses(n)
    local lz = (; offsets = offsets, residual = 0f0, rounds_used = 0)
    while true
        # working arrays = active (non-dropped) labels ∪ obstacle pseudo-nodes (all fixed)
        act = Int[i for i in 1:n if !dropped[i]]
        m = length(act); k = length(obstacles)
        w_anchors = Vector{Point2f}(undef, m + k)
        w_offsets = Vector{Vec2f}(undef, m + k)
        w_psizes  = Vector{Vec2f}(undef, m + k)
        w_fixed   = falses(m + k)
        for (t, i) in enumerate(act)
            w_anchors[t] = anchors[i]; w_offsets[t] = offsets[i]; w_psizes[t] = psizes[i]
            (pin_mask !== nothing && pin_mask[i]) && (w_fixed[t] = true)
        end
        for (t, ob) in enumerate(obstacles)
            w_anchors[m + t] = Point2f(ob.origin .+ ob.widths ./ 2)
            w_offsets[m + t] = Vec2f(0, 0)
            w_psizes[m + t]  = Vec2f(ob.widths)
            w_fixed[m + t]   = true
        end
        lz = legalize(w_anchors, w_offsets, w_psizes, bounds;
                      fixed = w_fixed, only_move = p.only_move)
        for (t, i) in enumerate(act)
            offsets[i] = lz.offsets[t]
        end
        (lz.residual ≤ 0.5f0 || count(!, dropped) ≤ 1) && break
        idx = drop_most_overlapped!(dropped, anchors, offsets, psizes, pin_mask)
        idx == 0 && break    # nothing eligible to drop (e.g. all survivors pinned) → stop, warn below
    end

    if lz.residual > 0.5f0
        @warn "ProjectionSolver: residual overlap after dropping; scene over-capacity for bounds=$bounds"
    end

    q = label_cost(anchors, sizes; offsets = offsets, bounds = bounds, dropped = dropped,
                   box_padding = p.box_padding, point_padding = p.point_padding,
                   min_segment_length = p.min_segment_length)
    s.stats[] = (; overlaps = q.overlaps, mean_leader = q.mean_leader, crossings = q.crossings,
                   iter = lz.rounds_used, residual = lz.residual, dropped = count(dropped))
    return (; offsets = offsets, dropped = dropped, iter = lz.rounds_used, residual = lz.residual)
end
```

- [ ] **Step 4: Wire the include**

In `src/MakieTextRepel.jl`, add `include("solvers/projection.jl")` immediately after `include("side_select.jl")` — so it loads after every pure layer it composes (`cost`, `legalize`, `side_select`) and after `init`/`voronoi`/`crossings`, and before `recipe.jl`/`annotation_algorithm.jl` consume it:

```julia
    include("legalize.jl")
    include("side_select.jl")
    include("solvers/projection.jl")
    include("measure.jl")
```

After all four tasks, the include block reads (in order): `geometry, solver, solvers/abstract, solvers/force, voronoi, init, connectors, crossings, cost, legalize, side_select, solvers/projection, measure, recipe, annotation_algorithm`.

- [ ] **Step 5: Run the test to verify it passes**

Run: `julia --project=. test/test_projection_solver.jl`
Expected: PASS — zero-overlap, leader ≤ force, drop loop, warm-start, pin preservation, determinism, and `drop_most_overlapped!` tie-break all green.

- [ ] **Step 6: Commit**

```bash
git add src/solvers/projection.jl src/MakieTextRepel.jl test/test_projection_solver.jl
git commit -m "feat: add ProjectionSolver (side-select → legalize) composing the pure layers"
```

---

## Task 5: Make `ProjectionSolver` the recipe default + zero-overlap invariant

**Files:**
- Modify: `src/recipe.jl:97`
- Modify: `test/test_integration.jl` (extend the invariants testset)
- Test: full suite via `Pkg.test()`

- [ ] **Step 1: Add the failing invariant assertion**

In `test/test_integration.jl`, the per-case testset (around lines 157–177) currently asserts a **hard** no-crossing guarantee: `@test isempty(find_crossings(connectors))` (line 176). Under v0.3 the hard guarantee shifts from no-crossing to **zero-overlap**: the pipeline runs `repair_crossings!` *before* the final `legalize`, and legalize's minimal nudge can re-introduce a crossing — so crossing-freeness becomes best-effort (no worse than ForceSolver), while zero-overlap is the new hard invariant.

First extend the import on line 133 from `using MakieTextRepel: connector_for, find_crossings` to:

```julia
using MakieTextRepel: connector_for, find_crossings, label_cost, solve_cluster, ForceSolver
```

Then **replace** the existing `@test isempty(find_crossings(connectors))` (line 176) with:

```julia
            # v0.3 HARD guarantee: zero box overlap under ProjectionSolver.
            q = label_cost(anchors, sizes; offsets = offsets, bounds = vp, dropped = dropped,
                           box_padding = params.box_padding,
                           point_padding = params.point_padding,
                           min_segment_length = min_len)
            @test q.overlaps == 0
            # Crossings are best-effort in v0.3 (repair precedes the final legalize):
            # assert no worse than the ForceSolver baseline on the same scene.
            rf = solve_cluster(ForceSolver(params), anchors, sizes, vp)
            force_conn = [connector_for(anchors[i], rf.offsets[i], sizes[i], rf.dropped[i], params, min_len)
                          for i in eachindex(rf.offsets)]
            @test length(find_crossings(connectors)) ≤ length(find_crossings(force_conn))
```

(`vp` is the case viewport already in scope at line 175's `within_bounds(anchors[i] + offsets[i], vp)`.)

> **If the `≤ force_conn` assertion fails on a dense fixture during implementation** (legalize re-introduced a crossing the force path avoided), that is the known best-effort limit. The documented refinement is to add a bounded post-legalize cleanup in `projection.jl` — `find_crossings`; if non-empty, one `repair_crossings!` + one `legalize` (overlap-safe, ends on legalize) — capped at 2 rounds. Add it only if a fixture actually trips the assertion; do not pre-emptively complicate the pipeline.

- [ ] **Step 2: Run the suite to verify the new assertion fails**

```bash
LOG="test/output/test-projection.log"
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -nA2 "overlaps == 0\|pipeline invariants" "$LOG"
```
Expected: the `q.overlaps == 0` assertion FAILS for at least one case — the recipe still uses `ForceSolver`, which leaves residual overlaps.

- [ ] **Step 3: Swap the recipe default**

In `src/recipe.jl`, change line 97 from:

```julia
        offsets, dropped, _, _ = solve_cluster(ForceSolver(params), anchors, sizes, bnds)
```
to:
```julia
        offsets, dropped, _, _ = solve_cluster(ProjectionSolver(params), anchors, sizes, bnds)
```

Update the comment on line 96 from "voronoi-init + force + repair" to:
```julia
        # Full placement strategy lives in the seam now (voronoi-seed → side-select →
        # crossing-repair → constraint-projection legalize).
```

- [ ] **Step 4: Run the suite to verify it passes**

```bash
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -E "Test Summary|Fail|Error" "$LOG"
```
Expected: the `q.overlaps == 0` invariant now PASSES for every case; the existing `find_crossings` and `within_bounds` invariants still pass. (Other testsets may report rebaseline failures handled in Task 7 — note them but they are expected at this point if Task 7 hasn't run.)

- [ ] **Step 5: Commit**

```bash
git add src/recipe.jl test/test_integration.jl
git commit -m "feat: ProjectionSolver is the textrepel! default; add zero-overlap invariant"
```

---

## Task 6: Make `ProjectionSolver` the annotation default + extend `solve_stats`

**Files:**
- Modify: `src/annotation_algorithm.jl` (struct fields, constructors, `solve_stats`, solver call, writeback)
- Test: `test/test_annotation_algorithm.jl`

- [ ] **Step 1: Write the failing test**

In `test/test_annotation_algorithm.jl`, add a testset asserting the extended `solve_stats` shape. (Place it after the existing stability-canary testset.)

```julia
using MakieTextRepel: solve_stats
@testset "solve_stats exposes Q diagnostics after a solve" begin
    alg = TextRepelAlgorithm()
    st0 = solve_stats(alg)
    @test st0.iter == 0 && st0.residual == 0f0
    @test st0.overlaps == 0 && st0.mean_leader == 0f0 && st0.crossings == 0 && st0.dropped == 0
    # field set is the v0.3 shape
    @test Set(keys(st0)) == Set((:iter, :residual, :overlaps, :mean_leader, :crossings, :dropped))
end
```

- [ ] **Step 2: Run to verify it fails**

```bash
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -nA2 "Q diagnostics" "$LOG"
```
Expected: FAIL — `solve_stats` returns only `(iter, residual)`; the `overlaps`/`keys` assertions error or fail.

- [ ] **Step 3: Extend the struct to hold a `ProjectionSolver` stats source**

In `src/annotation_algorithm.jl`, change the struct (lines ~46–48) so the algorithm owns a `ProjectionSolver` whose `stats` Ref is the single source of truth, replacing the two scalar Refs:

```julia
struct TextRepelAlgorithm
    params::RepelParams
    obstacles::Vector{Rect2f}
    solver::ProjectionSolver
end
```

Also refresh the now-stale prose in the type/module docstrings (the "force-directed solver" wording at `src/annotation_algorithm.jl:5–9` and `:32–34`) to say the algorithm uses MakieTextRepel's `ProjectionSolver` (side-select → legalize) and that `solve_stats` returns the Q diagnostics of the most recent solve.

Update both constructors (lines ~66 and ~75) to build the solver from `params`:

```julia
    return TextRepelAlgorithm(params, obstacles, ProjectionSolver(params))
```

(Apply the same one-line change at both constructor return sites.)

- [ ] **Step 4: Rewrite `solve_stats` to read the solver's Q Ref**

Replace the `solve_stats` definition (lines ~79–85) with:

```julia
"""
    solve_stats(alg::TextRepelAlgorithm) -> (; iter, residual, overlaps, mean_leader, crossings, dropped)

Diagnostics from the most recent solve. All zero before any solve runs.
"""
solve_stats(alg::TextRepelAlgorithm) = alg.solver.stats[]
```

- [ ] **Step 5: Use the owned solver in `calculate_best_offsets!` and drop the manual Ref writes**

In `calculate_best_offsets!`: the **all-pinned bypass** (the `if all(pin_mask)` block at `src/annotation_algorithm.jl:122–129`, which returns before any solve) currently sets `alg.last_iter[] = 0` / `alg.last_residual[] = 0f0`. Those Refs no longer exist. Replace the two lines with a single write of a zeroed Q tuple to the solver's Ref, so `solve_stats` reports a clean all-pinned result instead of stale values:

```julia
        alg.solver.stats[] = (; overlaps = 0, mean_leader = 0f0, crossings = 0,
                                iter = 0, residual = 0f0, dropped = 0)
        return
```

Change the solver call (line ~178) to solve on the algorithm's **owned** solver, so that `solve_cluster` writes Q into the very `stats` Ref that `solve_stats(alg)` reads. `effective_params` differs from `alg.params` only by `max_iter`, which `ProjectionSolver` ignores (it has no force loop), so calling on `alg.solver` directly is correct:

```julia
    r = solve_cluster(alg.solver, anchors, sizes, annotation_bounds;
                      init_state     = init_state,
                      pin_mask       = pin_mask,
                      pinned_offsets = pinned_solver,
                      obstacles      = alg.obstacles)
```

Then delete the two manual Ref writes that followed (the old `alg.last_iter[] = r.iter` / `alg.last_residual[] = r.residual`, ~lines 184–185) — `solve_cluster` already populated `alg.solver.stats[]` as a side effect, and `solve_stats` reads from there. (Grep the file for any other `last_iter`/`last_residual` reference and remove it; there should be none beyond the reset removed above and these two writes.)

- [ ] **Step 6: Run to verify it passes**

```bash
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -nA2 "Q diagnostics\|stability canary\|annotation" "$LOG"
```
Expected: the new `solve_stats` testset PASSES; the dispatch stability canary still passes (the registered `calculate_best_offsets!` signature is unchanged).

- [ ] **Step 7: Commit**

```bash
git add src/annotation_algorithm.jl test/test_annotation_algorithm.jl
git commit -m "feat: ProjectionSolver default for TextRepelAlgorithm; solve_stats exposes Q"
```

---

## Task 7: Rebaseline determinism + solver tests for the new default

**Files:**
- Modify: `test/test_solver.jl` (the v0.1 force-determinism baselines stay; ensure they target `ForceSolver` explicitly)
- Modify: `test/runtests.jl` (register the four new test files)
- Test: full suite

- [ ] **Step 1: Register the new test files**

In `test/runtests.jl`, insert the four NEW includes between the existing `include("test_crossings.jl")` (line 11) and the existing `include("test_annotation_algorithm.jl")` (line 12). Do **not** re-add `test_annotation_algorithm.jl` — it is already there. The block becomes:

```julia
    include("test_crossings.jl")
    include("test_cost.jl")
    include("test_legalize.jl")
    include("test_side_select.jl")
    include("test_projection_solver.jl")
    include("test_annotation_algorithm.jl")
```

(Only the four middle lines are new; the first and last already exist — this just shows the final ordering.)

- [ ] **Step 2: Run the full suite and capture every failure**

```bash
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -nB1 -A4 -i "fail\|error" "$LOG"
```
Expected: the only remaining failures are tests that asserted **force-solver-specific output** through the recipe/annotation default (e.g. a determinism test in `test_solver.jl` or `test_integration.jl` comparing recipe offsets to a hard-coded force baseline). List each.

- [ ] **Step 3: Repoint force-specific baselines at `ForceSolver` explicitly**

For each failing baseline that tests `solve_repel`/`ForceSolver` numerics **directly** (not through the recipe), it should already call `ForceSolver`/`solve_repel` by name — those must still pass unchanged (the force solver is untouched). For any test that asserted specific offset values **through the recipe default** (now `ProjectionSolver`), update it to one of:
  - construct `ForceSolver(params)` explicitly and assert the old values against it (if the intent was to test force numerics), **or**
  - replace the hard-coded offset values with the invariant the recipe now guarantees (zero overlap via `label_cost(...).overlaps == 0`, finite, in-bounds) if the intent was to test the recipe's output.

Apply the matching change per failing test identified in Step 2. Do not weaken `ForceSolver`'s own direct tests — they are the regression guard that the in-tree fallback still works.

- [ ] **Step 4: Run the full suite to verify green**

```bash
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -E "Test Summary|Fail|Error|Pass" "$LOG"
```
Expected: `Test Summary` shows 0 failures, 0 errors across all files including the four new ones.

- [ ] **Step 5: Commit**

```bash
git add test/runtests.jl test/test_solver.jl test/test_integration.jl
git commit -m "test: register new suites; rebaseline recipe defaults to ProjectionSolver"
```

---

## Task 8: Release prep — version, CHANGELOG, docstrings

**Files:**
- Modify: `Project.toml`
- Modify: `CHANGELOG.md`
- Modify: `src/recipe.jl` (docstring note on inert `force*`/`max_overlaps`)

- [ ] **Step 1: Bump the version**

In `Project.toml`, change `version = "0.2.0"` to `version = "0.3.0"`. (Verify the current value first with `grep '^version' Project.toml`; if it is not `0.2.0`, set it to one minor above whatever is there.)

- [ ] **Step 2: Add the CHANGELOG entry**

In `CHANGELOG.md`, add a new section above the most recent one:

```markdown
## [0.3.0]

- The default placement solver is now `ProjectionSolver`: discrete Imhof
  side-selection → crossing repair → Dykstra constraint-projection legalization.
  Layouts now have **zero box overlap** on feasible scenes and substantially
  shorter leader lines. Output positions differ from v0.2; existing caller code
  runs unchanged.
- Over-capacity scenes drop the most-overlapped labels deterministically until
  the remainder fits overlap-free (was: force-balance with residual overlaps).
- Leader-line crossings are now **best-effort reduced** rather than hard-guaranteed:
  v0.2's no-crossing guarantee is traded for the zero-overlap guarantee (the
  crossing-repair pass runs before the final legalize). Crossings remain no worse
  than the old force solver on the test fixtures.
- `force`, `force_point`, `force_pull`, and `max_overlaps` are now **inert** under
  the default solver (it has no force loop and guarantees zero overlap). They
  remain valid attributes and still drive the in-tree `ForceSolver`.
- `solve_stats(alg)` now returns `(; iter, residual, overlaps, mean_leader,
  crossings, dropped)` — the read-only Q quality functional — instead of just
  `(iter, residual)`.
```

- [ ] **Step 3: Document the behavior change in the recipe docstring**

In `src/recipe.jl`, locate the `@recipe TextRepel` attribute docstrings for `force`, `force_point`, `force_pull`, and `max_overlaps`. Append to each a sentence:

```
Inert under the default ProjectionSolver (no force loop / zero-overlap guarantee); affects only the in-tree ForceSolver.
```

- [ ] **Step 4: Verify the package still loads and the suite is green**

```bash
julia --project=. -e 'using MakieTextRepel; println("loads OK")'
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -E "Test Summary|Fail|Error" "$LOG"
```
Expected: `loads OK` and a fully green `Test Summary`.

- [ ] **Step 5: Commit**

```bash
git add Project.toml CHANGELOG.md src/recipe.jl
git commit -m "release: v0.3.0 — ProjectionSolver default, zero-overlap, Q diagnostics"
```

---

## Notes for the implementer

- **Determinism is a hard requirement.** Every new function is a pure function of its inputs with index-ordered iteration and no RNG. If you add a `Dict`/`Set`, iterate it only via membership tests (never order-dependent output), matching `repair_crossings!`'s `swapped::Set` precedent.
- **Keep the pure layers Makie-free.** `cost.jl`, `legalize.jl`, `side_select.jl` must not reference any `Makie.*` type — only GeometryBasics + module internals. `projection.jl` is the only new file that may touch `RepelParams`/solver internals.
- **The `bounds === nothing` determinism contract** (force solver) is untouched — `ProjectionSolver` always receives a real `Rect2f` from both surfaces, and `ForceSolver` is unchanged.
- **`label_cost` keyword shape:** note the spec's narrative gave a positional-`offsets` sketch; the implemented signature takes `offsets` as a **keyword** (`label_cost(anchors, sizes; offsets, bounds, …)`) so all call sites — `projection.jl`, `test_cost.jl`, `test_integration.jl` — must use the keyword form. This is the single canonical signature.
- **`legalize` obstacle handling:** the legalizer is agnostic to labels vs. obstacles; `projection.jl` appends obstacles as fixed pseudo-nodes (anchor = obstacle center, offset = 0, psize = obstacle widths, `fixed = true`) before the call and strips them on writeback. This realizes the spec's "working arrays = [labels; obstacles]".
- **Threshold ladder (deliberate, three values):** `0.01` px — legalizer convergence / constraint generation (`legalize.jl`); `0.5` px — Q overlap report, drop-loop stop, over-capacity `@warn` (`label_cost`, `projection.jl`). On a feasible scene the legalizer drives penetration below `0.01`, so its returned `residual` is ≈0 — far below the `0.5` report threshold — which is why `q.overlaps == 0` is robust against Float32 writeback noise (noise on px-scale offsets is ~1e-3, nowhere near the 0.49 gap). A `residual` in `(0.01, 0.5]` only arises on a marginal/over-capacity scene at the round cap, and the drop loop correctly treats it as "good enough, stop".
- **Legalizer is empirical, not a proof.** The per-round single-axis (cheaper-axis) assignment + Dykstra projection + bounds clamp is a relaxation validated to reach zero overlap on the §7c feasible fixtures — it is **not** a proven global QP optimum, and the post-hoc bounds clamp can fail to fully converge on a *pathologically bounds-tight* packing (boxes that fit only in a specific corner arrangement). Such a scene registers as `residual > 0.5` → the drop loop sheds a label. That is acceptable for v1 (the real scan-line VPSC that treats bounds as constraints is the deferred upgrade per the spec's Non-goals). Do not claim "provably zero overlap" anywhere in code comments.
- **Warm-start no-op preservation** holds only when the supplied `init_state` is separated by `> 0.01 px` (the legalizer threshold). A caller warm-starting from a layout with sub-0.01px touches will see those pairs nudged. The `test_projection_solver` warm-start fixtures are separated by tens of px, well clear of this.
