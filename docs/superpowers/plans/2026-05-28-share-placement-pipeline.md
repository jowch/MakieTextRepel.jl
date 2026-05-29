# Share Placement Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `ForceSolver.solve_cluster` own the entire v0.2 placement strategy (Voronoi-init → force → crossing-repair, pin-aware) so both `textrepel!` and `TextRepelAlgorithm` reach it through one call, and any future strategy change benefits both surfaces.

**Architecture:** Promote the `AbstractClusterSolver` seam to own the strategy. The strategy's "fresh vs. relax" axis is encoded as `init_state` (`nothing` ⇒ fresh = voronoi-init + repair; given ⇒ relax = solve-only). Pinned labels are seeded in init, held by the solver, and skipped by repair — an absolute guarantee. `min_segment_length` moves into `RepelParams` so the strategy is fully described by `ForceSolver(params)`. The recipe and `calculate_best_offsets!` each collapse to a single `solve_cluster` call; the annotation surface keeps only its `align_bias` coordinate translation as a thin adapter.

**Tech Stack:** Julia 1.11, Makie 0.24, GeometryBasics, DelaunayTriangulation (via `voronoi_cells`). Tests use `Test` + CairoMakie. Pure layers are Makie-free.

**Spec:** `docs/superpowers/specs/2026-05-28-share-placement-pipeline-design.md`

**Testing note (from CLAUDE.md):** `Pkg.test()` costs minutes (precompilation). Run the full suite ONCE per code change, tee to an agent-scoped log under `test/output/`, then `grep` the log. Pick one slug for the session:

```bash
LOG="test/output/test-$(whoami)-issue12.log"
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -E "Test Summary|Fail|Error" "$LOG"
```

For fast single-layer iteration on the pure functions (no backend needed), use the package env REPL:

```bash
julia --project=. -i -e 'using MakieTextRepel'
# then e.g. MakieTextRepel.solve_cluster(...), MakieTextRepel.initial_offsets(...)
```

These pure-layer symbols are not exported; access them as `MakieTextRepel.name` or `using MakieTextRepel: name`.

---

## File structure

| File | Responsibility | Change |
|---|---|---|
| `src/solver.jl` | `RepelParams`, `solve_repel`, `init_offsets` | **Modify**: add `min_segment_length` field to `RepelParams` |
| `src/init.jl` | `initial_offsets` (Imhof slot init) | **Modify**: optional `pin_mask`/`pinned_offsets` kwargs |
| `src/crossings.jl` | `repair_crossings!` (swap-based repair) | **Modify**: optional `pin_mask` kwarg, skip pinned pairs |
| `src/solvers/abstract.jl` | `AbstractClusterSolver` contract docstring | **Modify**: document new signature/return |
| `src/solvers/force.jl` | `ForceSolver.solve_cluster` | **Modify**: own the whole strategy (the new contract) |
| `src/recipe.jl` | `textrepel!` recipe | **Modify**: wire `min_segment_length` into params; collapse `89-92` to one call |
| `src/annotation_algorithm.jl` | `calculate_best_offsets!` | **Modify**: delete spiral/psizes block; call `solve_cluster` |
| `test/test_solver.jl` | solver + seam contract tests | **Modify**: rewrite `solve_cluster` contract test; add relax/obstacles |
| `test/test_init.jl` | init tests | **Modify**: add pin-seeding tests |
| `test/test_crossings.jl` | repair tests | **Modify**: add pin-skip test |
| `test/test_integration.jl` | recipe end-to-end | **Modify**: add recipe == manual-pipeline byte-identity test |
| `test/test_annotation_algorithm.jl` | annotation path tests | **Modify**: fresh crossing-free, determinism, non-centered, all-pinned, warm-start |

No new source files. Tasks are ordered so the package always loads and the suite stays green at each commit — **except** Task 4, where the breaking `solve_cluster` signature change and its only caller (recipe) + contract test are updated atomically in one commit.

---

### Task 1: Add `min_segment_length` to `RepelParams`

The strategy's notion of minimum meaningful leader length. Default **must** be `2.0` to match the recipe's `@recipe` attribute default (`src/recipe.jl:31`), or recipe output changes. The `@kwdef` copy-with-overrides constructor (`src/solver.jl:25-29`) enumerates `fieldnames` dynamically, so it carries the new field automatically — no constructor edits needed.

**Files:**
- Modify: `src/solver.jl:4-17` (struct body)
- Test: `test/test_solver.jl`

- [ ] **Step 1: Write the failing test**

Add to `test/test_solver.jl` (near the other `RepelParams` tests around line 43):

```julia
@testset "RepelParams min_segment_length field" begin
    @test RepelParams().min_segment_length == 2.0          # default matches recipe attr
    base = RepelParams()
    @test RepelParams(base; min_segment_length = 7.0).min_segment_length == 7.0
    # copy-with-overrides carries the field when overriding something else:
    @test RepelParams(base; max_iter = 5).min_segment_length == 2.0
end
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `julia --project=. -i -e 'using MakieTextRepel: RepelParams; @show hasfield(RepelParams, :min_segment_length)'`
Expected: prints `false` (field does not exist yet; the testset would error on `.min_segment_length`).

- [ ] **Step 3: Add the field**

In `src/solver.jl`, add the field to the `@kwdef struct RepelParams` body. Place it after `tol` and before `bounds`:

```julia
    tol::Float64                    = 0.1           # convergence: max move < tol
    min_segment_length::Float64     = 2.0           # min meaningful leader length (repair threshold); matches recipe attr
    bounds::Union{Rect2f, Nothing} = nothing   # clamp region in solver (pixel) space; nothing = no clamp
```

- [ ] **Step 4: Verify it passes**

Run: `julia --project=. -i -e 'using MakieTextRepel: RepelParams; @assert RepelParams().min_segment_length == 2.0; @assert RepelParams(RepelParams(); max_iter=5).min_segment_length == 2.0; println("ok")'`
Expected: prints `ok`.

- [ ] **Step 5: Commit**

```bash
git add src/solver.jl test/test_solver.jl
git commit -m "Add min_segment_length field to RepelParams (#12)"
```

---

### Task 2: Pin-aware `initial_offsets`

`initial_offsets` (`src/init.jl:36-61`) currently picks an Imhof slot for every label. Add optional `pin_mask`/`pinned_offsets` kwargs: when `pin_mask === nothing` (the recipe path) behavior is **byte-identical**; when provided, pinned indices are seeded at their pinned offset instead of a slot.

**Files:**
- Modify: `src/init.jl:36-61`
- Test: `test/test_init.jl`

- [ ] **Step 1: Write the failing test**

Add to `test/test_init.jl`:

```julia
@testset "initial_offsets honors pinned indices" begin
    anchors = [Point2f(0, 0), Point2f(50, 0), Point2f(0, 50)]
    sizes   = [Vec2f(10, 6), Vec2f(10, 6), Vec2f(10, 6)]
    cells   = Union{GeometryBasics.Polygon, Nothing}[nothing, nothing, nothing]
    params  = RepelParams()

    # No pin args → identical to the bare call (byte-identity for the recipe path).
    base = MakieTextRepel.initial_offsets(anchors, sizes, cells, params)
    same = MakieTextRepel.initial_offsets(anchors, sizes, cells, params;
                                          pin_mask = nothing, pinned_offsets = Vec2f[])
    @test same == base

    # Pin index 2 at a specific offset → that index is seeded there, others unchanged.
    pin_mask = BitVector([false, true, false])
    pinned   = [Vec2f(0, 0), Vec2f(99, 99), Vec2f(0, 0)]
    out = MakieTextRepel.initial_offsets(anchors, sizes, cells, params;
                                         pin_mask = pin_mask, pinned_offsets = pinned)
    @test out[2] == Vec2f(99, 99)
    @test out[1] == base[1]
    @test out[3] == base[3]
end
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `julia --project=. -i -e 'using MakieTextRepel, GeometryBasics; using MakieTextRepel: RepelParams; MakieTextRepel.initial_offsets(Point2f[], Vec2f[], Union{GeometryBasics.Polygon,Nothing}[], RepelParams(); pin_mask=nothing)'`
Expected: `MethodError` / unexpected keyword argument `pin_mask` (kwarg not accepted yet).

- [ ] **Step 3: Add the kwargs**

In `src/init.jl`, change the signature and add a seed loop. Replace the function head and the return:

```julia
function initial_offsets(anchors::Vector{Point2f}, sizes::Vector{Vec2f},
                         cells::Vector{<:Union{GeometryBasics.Polygon, Nothing}},
                         params;
                         pin_mask::Union{Nothing,BitVector} = nothing,
                         pinned_offsets::Vector{Vec2f}       = Vec2f[])
    n = length(anchors)
    offsets = Vector{Vec2f}(undef, n)
    pad = Float32(params.box_padding)
    p = params.point_padding
    for i in 1:n
        if pin_mask !== nothing && pin_mask[i]
            offsets[i] = pinned_offsets[i]      # pinned: seed at the fixed offset, skip slot search
            continue
        end
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

- [ ] **Step 4: Verify the focused tests pass**

Run the suite once (per the Testing note) and grep:

```bash
LOG="test/output/test-$(whoami)-issue12.log"
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -nA2 "initial_offsets honors pinned" "$LOG"
grep -E "Test Summary|Fail|Error" "$LOG"
```
Expected: the new testset passes; no new failures elsewhere.

- [ ] **Step 5: Commit**

```bash
git add src/init.jl test/test_init.jl
git commit -m "Teach initial_offsets to honor pinned indices (#12)"
```

---

### Task 3: Pin-aware `repair_crossings!`

`repair_crossings!` (`src/crossings.jl:65-91`) swaps crossing pairs. Add optional `pin_mask` kwarg: skip any crossing pair where **either** member is pinned (pinned offsets are never moved). `min_len` stays a **required** kwarg so existing callers compile unchanged. When `pin_mask === nothing`, no pairs are skipped → byte-identical.

**Files:**
- Modify: `src/crossings.jl:65-80`
- Test: `test/test_crossings.jl`

- [ ] **Step 1: Write the failing test**

Add to `test/test_crossings.jl`. This builds two labels whose leaders cross, then pins one and asserts neither moves:

```julia
@testset "repair_crossings! skips pairs touching a pinned label" begin
    # Two anchors with offsets that make their leaders cross (label i sits over
    # anchor j's side and vice versa).
    anchors = [Point2f(0, 0), Point2f(20, 0)]
    sizes   = [Vec2f(6, 4), Vec2f(6, 4)]
    dropped = falses(2)
    params  = RepelParams(point_padding = 0.0)

    crossed = [Vec2f(20, 10), Vec2f(-20, 10)]   # i reaches right, j reaches left → cross

    # Without pins: the pair gets swapped (offsets change).
    off_free = copy(crossed)
    MakieTextRepel.repair_crossings!(off_free, anchors, sizes, dropped, params; min_len = 0.0)
    @test off_free != crossed

    # Pin label 1: the crossing pair (1,2) touches a pinned label → skipped, nothing moves.
    off_pin  = copy(crossed)
    pin_mask = BitVector([true, false])
    MakieTextRepel.repair_crossings!(off_pin, anchors, sizes, dropped, params;
                                     min_len = 0.0, pin_mask = pin_mask)
    @test off_pin == crossed
end
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `julia --project=. -i -e 'using MakieTextRepel; using MakieTextRepel: RepelParams; MakieTextRepel.repair_crossings!(Vec2f[], Point2f[], Vec2f[], falses(0), RepelParams(); min_len=0.0, pin_mask=nothing)'`
Expected: `MethodError` / unexpected keyword argument `pin_mask`.

- [ ] **Step 3: Add the kwarg and the skip guard**

In `src/crossings.jl`, change the signature and the swap loop:

```julia
function repair_crossings!(offsets::Vector{Vec2f}, anchors::Vector{Point2f},
                           sizes::Vector{Vec2f}, dropped::BitVector,
                           params; min_len::Real, max_iter::Int = 100,
                           pin_mask::Union{Nothing,BitVector} = nothing)
    is_pinned(k) = pin_mask !== nothing && pin_mask[k]
    for iter in 1:max_iter
        connectors = [connector_for(anchors[i], offsets[i], sizes[i], dropped[i], params, min_len)
                      for i in eachindex(offsets)]
        crossings = find_crossings(connectors)
        isempty(crossings) && return iter - 1

        swapped = Set{Int}()
        for (i, j) in crossings
            (i in swapped || j in swapped) && continue
            (is_pinned(i) || is_pinned(j)) && continue   # never move a pinned label
            swap_positions!(offsets, anchors, i, j)
            push!(swapped, i)
            push!(swapped, j)
        end
    end
```

(Leave the final rescan/`@warn` block below unchanged.)

- [ ] **Step 4: Verify the focused tests pass**

```bash
LOG="test/output/test-$(whoami)-issue12.log"
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -nA2 "skips pairs touching a pinned" "$LOG"
grep -E "Test Summary|Fail|Error" "$LOG"
```
Expected: new testset passes; existing `test_crossings.jl` callers (which pass only `min_len`) still pass.

- [ ] **Step 5: Commit**

```bash
git add src/crossings.jl test/test_crossings.jl
git commit -m "Teach repair_crossings! to skip pinned pairs (#12)"
```

---

### Task 4: `solve_cluster` owns the strategy + recipe collapse + contract test

**This is the core, breaking-change task — keep it atomic.** Changing `solve_cluster`'s signature breaks its only `src/` caller (the recipe) and its contract test, so all three change in one commit and the package must load + the suite stay green at the end.

`solve_cluster`'s new contract: `bounds` becomes positional (4th arg), `initial_offsets` is dropped, init/repair move inside, return widens to a named tuple, and `init_state` encodes fresh-vs-relax.

**Files:**
- Modify: `src/solvers/abstract.jl:3-8` (docstring)
- Modify: `src/solvers/force.jl:8-15` (the method)
- Modify: `src/recipe.jl:81-86` (wire `min_segment_length`) and `89-92` (collapse)
- Modify: `test/test_solver.jl:347-359` (rewrite contract test) + add relax/obstacles tests

- [ ] **Step 1: Write the failing/updated contract tests**

Replace the existing `@testset "ForceSolver wraps solve_repel"` block in `test/test_solver.jl` (lines 347-359) with:

```julia
@testset "solve_cluster owns the strategy (fresh vs relax)" begin
    anchors = [Point2f(0, 0), Point2f(20, 0), Point2f(0, 20)]
    sizes   = [Vec2f(6, 4), Vec2f(6, 4), Vec2f(6, 4)]
    bounds  = Rect2f(-50, -50, 100, 100)
    params  = RepelParams()
    solver  = ForceSolver(params)

    # FRESH (init_state === nothing): equals the explicit voronoi→init→solve→repair
    # pipeline that the recipe used to inline. This is the structural-defense test.
    r = solve_cluster(solver, anchors, sizes, bounds)
    @test r isa NamedTuple
    @test propertynames(r) == (:offsets, :dropped, :iter, :residual)

    p     = RepelParams(params; bounds = bounds)
    cells = MakieTextRepel.voronoi_cells(anchors, bounds)
    init  = MakieTextRepel.initial_offsets(anchors, sizes, cells, p)
    manual = solve_repel(anchors, sizes, p; init_state = init)
    expected = copy(manual.offsets)
    MakieTextRepel.repair_crossings!(expected, anchors, sizes, manual.dropped, p;
                                     min_len = p.min_segment_length)
    @test r.offsets == expected
    @test r.dropped == manual.dropped

    # RELAX (init_state given): solve-only — no voronoi, no repair. Equals a direct
    # solve_repel with the SAME warm init and aux kwargs.
    warm = [Vec2f(5, 5), Vec2f(-5, 5), Vec2f(5, -5)]
    rr   = solve_cluster(solver, anchors, sizes, bounds; init_state = warm)
    direct = solve_repel(anchors, sizes, p; init_state = warm)
    @test rr.offsets == direct.offsets
    @test rr.dropped == direct.dropped
end

@testset "solve_cluster forwards obstacles" begin
    anchors = [Point2f(0, 0)]
    sizes   = [Vec2f(10, 6)]
    bounds  = Rect2f(-100, -100, 200, 200)
    ob      = Rect2f(5, -10, 30, 20)          # obstacle to the right of the anchor
    solver  = ForceSolver(RepelParams())
    r = solve_cluster(solver, anchors, sizes, bounds; obstacles = [ob])
    # The label box must not overlap the obstacle after solving.
    box = MakieTextRepel.box_at(anchors[1], r.offsets[1], sizes[1] .+ 2*4.0f0)
    @test MakieTextRepel.overlap_push(box, ob) == Vec2f(0, 0)
end
```

- [ ] **Step 2: Run to verify it fails**

Run: `julia --project=. -i -e 'using MakieTextRepel: ForceSolver, solve_cluster, RepelParams; solve_cluster(ForceSolver(RepelParams()), [Point2f(0,0)], [Vec2f(6,4)], Rect2f(-10,-10,20,20))'`
Expected: `MethodError` — no method `solve_cluster(::ForceSolver, ::Vector{Point2f}, ::Vector{Vec2f}, ::Rect2f)` (current method needs 5 positional args).

- [ ] **Step 3: Rewrite `solve_cluster` in `src/solvers/force.jl`**

Replace the function body (lines 8-15) with:

```julia
function solve_cluster(s::ForceSolver, anchors::Vector{Point2f}, sizes::Vector{Vec2f},
                       bounds::Rect2f;
                       init_state::Union{Nothing,Vector{Vec2f}} = nothing,
                       pin_mask::Union{Nothing,BitVector}        = nothing,
                       pinned_offsets::Vector{Vec2f}             = Vec2f[],
                       obstacles::Vector{Rect2f}                 = Rect2f[])
    # `RepelParams(base; ...)` (src/solver.jl:25-29) copies s.params, overriding bounds.
    p = RepelParams(s.params; bounds = bounds)
    fresh = init_state === nothing
    init = fresh ?
        initial_offsets(anchors, sizes, voronoi_cells(anchors, bounds), p;
                        pin_mask = pin_mask, pinned_offsets = pinned_offsets) :
        init_state
    r = solve_repel(anchors, sizes, p;
                    init_state = init, obstacles = obstacles,
                    pin_mask = pin_mask, pinned_offsets = pinned_offsets)
    if fresh
        repair_crossings!(r.offsets, anchors, sizes, r.dropped, p;
                          min_len = p.min_segment_length, pin_mask = pin_mask)
    end
    return (; offsets = r.offsets, dropped = r.dropped, iter = r.iter, residual = r.residual)
end
```

- [ ] **Step 4: Update the `AbstractClusterSolver` docstring**

In `src/solvers/abstract.jl`, replace the docstring (lines 3-8) so the advertised contract matches:

```julia
"""
Marker type for cluster placement solvers. A concrete subtype owns the **entire**
placement strategy and implements

    solve_cluster(s, anchors, sizes, bounds;
                  init_state = nothing, pin_mask = nothing,
                  pinned_offsets = Vec2f[], obstacles = Rect2f[])
        -> (; offsets::Vector{Vec2f}, dropped::BitVector, iter::Int, residual::Float32)

`init_state === nothing` ⇒ fresh placement (the solver does its own init + crossing
repair); a given `init_state` ⇒ relax (warm-start, solve only). Callers must NOT
perform init/placement/repair outside `solve_cluster`. Internal in v0.2; exposed
publicly when a second implementation lands (see GitHub issue #8).
"""
```

- [ ] **Step 5: Collapse the recipe call site**

In `src/recipe.jl`, two edits inside the `solved = lift(...)` body.

(a) Add `min_segment_length` to the `RepelParams` construction (lines 81-86). `ml` is already a closure variable (it is the last `lift` arg). Replace the constructor call with:

```julia
        params = RepelParams(; force = Tuple(Float64.(fr)),
                               force_point = Tuple(Float64.(frp)),
                               force_pull = Tuple(Float64.(fpl)),
                               max_iter = Int(mi), only_move = Symbol(om),
                               box_padding = Float64(bp), point_padding = Float64(pp),
                               max_overlaps = Float64(mo), bounds = bnds,
                               min_segment_length = Float64(ml))
```

(b) Replace the four lines `89-92` (the `cells`/`init`/`solve_cluster`/`repair_crossings!` block) with a single call. Leave line 93's returned tuple unchanged:

```julia
        # Full placement strategy lives in the seam now (voronoi-init + force + repair).
        offsets, dropped, _, _ = solve_cluster(ForceSolver(params), anchors, sizes, bnds)
        (; anchors, sizes, offsets, dropped, params)
```

(`offsets, dropped, _, _` positionally destructures the NamedTuple in field order `offsets, dropped, iter, residual`; `_` discards the last two. The downstream `lift`s still consume `s.anchors/s.sizes/s.params/s.dropped` from the returned tuple.)

- [ ] **Step 6: Run the full suite**

```bash
LOG="test/output/test-$(whoami)-issue12.log"
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -E "Test Summary|Fail|Error" "$LOG"
grep -nA2 "solve_cluster owns the strategy\|forwards obstacles" "$LOG"
```
Expected: package loads; the two new solver testsets pass; the existing recipe determinism + axis-limit regression tests in `test_integration.jl` still pass (recipe output is byte-identical — same voronoi→init→solve→repair, same `min_len`).

- [ ] **Step 7: Commit**

```bash
git add src/solvers/force.jl src/solvers/abstract.jl src/recipe.jl test/test_solver.jl
git commit -m "solve_cluster owns the placement strategy; recipe calls the seam (#12)"
```

---

### Task 5: Recipe byte-identity guard (structural defense)

Lock in that the recipe's solve output equals a direct `solve_cluster` call, so a future inline strategy leak fails CI. This is structural-defense test #1 from the spec.

**Files:**
- Test: `test/test_integration.jl`

- [ ] **Step 1: Write the test**

Add to `test/test_integration.jl` (it already exercises the recipe; reuse its style). This drives the recipe and compares its `computed_offsets` against a direct seam call with the same params:

```julia
@testset "recipe solve equals a direct solve_cluster call (#12 structural defense)" begin
    fig = Figure()
    ax  = Axis(fig[1, 1])
    pts = [Point2f(1, 1), Point2f(2, 2), Point2f(1.5, 2.5), Point2f(2.2, 1.1)]
    plt = textrepel!(ax, pts; text = ["alpha", "beta", "gamma", "delta"])
    Makie.update_state_before_display!(fig)   # force the lift graph to compute

    anchors = plt.attributes[:computed_anchors][]
    sizes   = plt.attributes[:computed_sizes][]
    params  = plt.attributes[:computed_params][]
    direct  = MakieTextRepel.solve_cluster(MakieTextRepel.ForceSolver(params),
                                           anchors, sizes, params.bounds)
    @test plt.attributes[:computed_offsets][] == direct.offsets
    @test plt.attributes[:computed_dropped][] == direct.dropped
end
```

- [ ] **Step 2: Run to verify it passes (no implementation needed — guard test)**

```bash
LOG="test/output/test-$(whoami)-issue12.log"
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -nA3 "structural defense" "$LOG"
grep -E "Test Summary|Fail|Error" "$LOG"
```
Expected: PASS. (If it fails, the recipe is doing placement work outside the seam — investigate before proceeding.)

- [ ] **Step 3: Commit**

```bash
git add test/test_integration.jl
git commit -m "Guard: recipe solve output equals solve_cluster (#12)"
```

---

### Task 6: Collapse the annotation call site onto the seam

Delete the `psizes`/spiral init block and the direct `solve_repel` call in `calculate_best_offsets!` (`src/annotation_algorithm.jl`, span lines 174-187), and call `solve_cluster` instead. Keep the all-pinned bypass, the `align_bias` translation, `pinned_solver`, and the writeback. Fresh (`reset=true`) passes `init_state = nothing` and inherits voronoi-init + repair; relax (`reset=false`) passes the warm offsets. This also removes the pre-existing double-padding (`psizes` fed to a self-padding init).

**Files:**
- Modify: `src/annotation_algorithm.jl:174-190`
- Test: `test/test_annotation_algorithm.jl`

- [ ] **Step 1: Write the failing tests**

Add to `test/test_annotation_algorithm.jl`. These call the registered hook directly (the file already uses this pattern — see its stability canary). Helper to build inputs for centered text:

```julia
@testset "annotation fresh path inherits crossing-free repair (#12)" begin
    # A layout that, under plain golden-angle init + no repair, produced crossing
    # leaders. After routing through solve_cluster (voronoi-init + repair) it must not.
    anchors = [Point2f(0, 0), Point2f(40, 0), Point2f(0, 40), Point2f(40, 40)]
    n = length(anchors)
    bbox = Rect2f(-100, -100, 300, 300)
    # centered text bbs: origin = anchor - widths/2 → align_bias = 0
    w = Vec2f(20, 10)
    text_bbs = [Rect2f(a[1]-w[1]/2, a[2]-w[2]/2, w[1], w[2]) for a in anchors]
    tpos_off = fill(Point2f(NaN, NaN), n)        # all auto-placed
    offsets  = fill(Vec2f(0, 0), n)

    alg = TextRepelAlgorithm()
    Makie.calculate_best_offsets!(alg, offsets, anchors, tpos_off, text_bbs, bbox;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)

    # Build leader connectors at the solved positions and assert none cross.
    sizes = [Vec2f(bb.widths...) for bb in text_bbs]
    conns = [MakieTextRepel.connector_for(anchors[i], Vec2f(offsets[i]...), sizes[i],
                                          false, RepelParams(), 0.0) for i in 1:n]
    @test isempty(MakieTextRepel.find_crossings(conns))
end

@testset "annotation fresh path is deterministic (#12)" begin
    anchors = [Point2f(0,0), Point2f(10,0), Point2f(10,0), Point2f(0,10)]  # incl. coincident
    n = length(anchors)
    bbox = Rect2f(-100, -100, 300, 300)
    w = Vec2f(16, 8)
    text_bbs = [Rect2f(a[1]-w[1]/2, a[2]-w[2]/2, w[1], w[2]) for a in anchors]
    tpos_off = fill(Point2f(NaN, NaN), n)

    run() = (o = fill(Vec2f(0,0), n);
             Makie.calculate_best_offsets!(TextRepelAlgorithm(), o, anchors, tpos_off,
                 text_bbs, bbox; maxiter = Makie.automatic, labelspace = :relative_pixel,
                 reset = true); o)
    o1 = run(); o2 = run()
    @test o1 == o2                       # seeded voronoi → identical across runs
    # coincident anchors (2,3) must not collapse onto the same rendered position:
    @test anchors[2] .+ o1[2] != anchors[3] .+ o1[3]
end

@testset "annotation non-centered text places sanely (#12, dismissed Risk 3)" begin
    # Left/bottom-aligned text: bbox origin AT the anchor → align_bias = widths/2 ≠ 0.
    anchors = [Point2f(0, 0), Point2f(30, 0), Point2f(0, 30)]
    n = length(anchors)
    bbox = Rect2f(-100, -100, 300, 300)
    w = Vec2f(20, 10)
    text_bbs = [Rect2f(a[1], a[2], w[1], w[2]) for a in anchors]   # origin = anchor
    tpos_off = fill(Point2f(NaN, NaN), n)
    offsets  = fill(Vec2f(0, 0), n)

    alg = TextRepelAlgorithm()
    Makie.calculate_best_offsets!(alg, offsets, anchors, tpos_off, text_bbs, bbox;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)

    # All offsets finite, and each label is pushed off its own anchor (not collapsed
    # onto it) — own-anchor repulsion holds regardless of alignment.
    for i in 1:n
        @test all(isfinite, offsets[i])
        rendered_center = anchors[i] .+ Vec2f(offsets[i]...) .+ w ./ 2
        @test rendered_center != anchors[i]
    end
end

@testset "annotation all-pinned bypass preserved (#12)" begin
    anchors = [Point2f(0,0), Point2f(20,0)]
    n = length(anchors)
    bbox = Rect2f(-50,-50,100,100)
    w = Vec2f(10,6)
    text_bbs = [Rect2f(a[1]-w[1]/2, a[2]-w[2]/2, w[1], w[2]) for a in anchors]
    # all finite textpositions_offset → all pinned. Pinned render offset = tpo - anchor.
    tpos_off = [Point2f(3, 3), Point2f(23, -4)]
    offsets  = fill(Vec2f(0,0), n)
    alg = TextRepelAlgorithm()
    Makie.calculate_best_offsets!(alg, offsets, anchors, tpos_off, text_bbs, bbox;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)
    @test offsets[1] == Vec2f(3, 3)
    @test offsets[2] == Vec2f(3, -4)
    @test alg.last_iter[] == 0           # bypass: solver never ran
end
```

- [ ] **Step 2: Run to verify the new tests fail (or error)**

```bash
LOG="test/output/test-$(whoami)-issue12.log"
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -nA3 "inherits crossing-free\|is deterministic\|all-pinned bypass preserved" "$LOG"
```
Expected: the crossing-free and determinism testsets FAIL (current annotation path uses spiral init, no repair, so leaders may cross / placement differs). The all-pinned testset should already PASS (bypass exists). This confirms the new behavior is not yet present.

- [ ] **Step 3: Rewrite the solve block in `calculate_best_offsets!`**

In `src/annotation_algorithm.jl`, replace the `init_state` block + `solve_repel` call + diagnostics writeback (current lines 168-190) with:

```julia
    # Initial state for the seam:
    # - reset=true: fresh placement → pass `nothing`; solve_cluster does voronoi-init
    #   (Imhof slots, in solver-space) + crossing-repair. No align_bias needed on the
    #   init: the writeback below subtracts align_bias, and rendered box-center =
    #   anchor + solver_offset for any alignment.
    # - reset=false: warm-start from the previous render-space offsets, translated to
    #   solver-space by adding align_bias.
    init_state = reset ? nothing :
                 Vec2f[Vec2f(offsets[i][1], offsets[i][2]) + align_bias[i] for i in 1:n]

    r = solve_cluster(ForceSolver(effective_params), anchors, sizes, annotation_bounds;
                      init_state     = init_state,
                      pin_mask       = pin_mask,
                      pinned_offsets = pinned_solver,
                      obstacles      = alg.obstacles)

    alg.last_iter[]     = r.iter
    alg.last_residual[] = r.residual

    # Writeback: solver-space → render-space (subtract align_bias). Pinned indices
    # recover pinned_render[i] exactly (solve_repel holds them at pinned_solver[i]).
    for i in 1:n
        o = r.offsets[i] .- align_bias[i]
        if all(isfinite, o)
            offsets[i] = T(o[1], o[2])
        else
            offsets[i] = T(0, 0)
        end
    end
    return
end
```

Confirm the now-unused locals are gone: the `psizes`/`spiral`/`if reset … else … end` block (old 174-181) and the old `solve_repel(...)` call (old 183-187) are fully replaced by the above. `effective_params`, `align_bias`, `pinned_solver`, the all-pinned bypass, and `T = eltype(offsets)` all remain as they were.

- [ ] **Step 4: Run the full suite**

```bash
LOG="test/output/test-$(whoami)-issue12.log"
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -E "Test Summary|Fail|Error" "$LOG"
grep -nA3 "inherits crossing-free\|is deterministic\|all-pinned bypass preserved" "$LOG"
```
Expected: all three new annotation testsets pass; the existing annotation tests (warm-start, pinning, obstacles, stability canary) still pass.

- [ ] **Step 5: Commit**

```bash
git add src/annotation_algorithm.jl test/test_annotation_algorithm.jl
git commit -m "Route annotation path through solve_cluster; inherits voronoi-init + repair (closes #12)"
```

---

### Task 7: Full-suite verification & spec cross-check

- [ ] **Step 1: Run the complete suite once more from clean**

```bash
LOG="test/output/test-$(whoami)-issue12-final.log"
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -E "Test Summary|Fail|Error" "$LOG"
```
Expected: zero failures/errors across all 8 test files.

- [ ] **Step 2: Cross-check the spec test plan**

Confirm each spec test-plan item maps to a test that now exists and passes:
1. Recipe byte-identity → Task 5 + existing determinism test. 2. Pinning×repair → Tasks 2,3. 3. Relax skips voronoi/repair → Task 4 relax test. 4. Fresh crossing-free → Task 6. 5. Coincident → Task 6 determinism test; non-centered → Task 6 "places sanely" test. 6. Determinism → Task 6. 7. Obstacles through seam → Task 4. 8. All-pinned bypass → Task 6. 9. min_segment_length → Task 1. 10. Seam canary → Task 4. 11. repair_crossings! sig → Task 3. 12. Dispatch canary → unchanged, still in suite.

- [ ] **Step 3: Final commit if any gap-filling test was added**

```bash
git add -A
git commit -m "Tests: fill remaining #12 spec test-plan items"
```

---

## Self-review notes

- **DRY:** the four strategy steps now live in exactly one place (`ForceSolver.solve_cluster`); both surfaces call it.
- **YAGNI:** no new abstraction layer beyond the existing seam; `min_len` stays a required kwarg on `repair_crossings!` rather than over-generalizing.
- **Byte-identity:** guarded by Task 5 + the existing recipe determinism/axis-limit tests; `min_segment_length` default pinned to `2.0` (Task 1) so the recipe threshold is unchanged.
- **Determinism contract:** `solve_repel`'s `bounds === nothing` path is never exercised by these changes (both surfaces always pass a `Rect2f`).
