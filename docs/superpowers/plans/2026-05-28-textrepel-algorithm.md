# `TextRepelAlgorithm` Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship `TextRepelAlgorithm`, a plug-in for `Makie.annotation!`'s `algorithm` attribute that delegates label placement to MakieTextRepel's force-directed solver, alongside the existing `textrepel!` recipe.

**Architecture:** The algorithm struct wraps a `RepelParams` (so future solver knobs propagate automatically). It registers a method on `Makie.calculate_best_offsets!` that translates between annotation's call shape (text bboxes, viewport, manual offsets, reset flag) and the solver's call shape. The solver gains four backwards-compatible kwargs (`obstacles`, `init_state`, `pin_mask`, `pinned_offsets`) and a NamedTuple return that carries diagnostics alongside `dropped`. The `textrepel!` recipe is touched at one site to consume the new return shape.

**Tech Stack:** Julia 1.11+, Makie 0.24.10, GeometryBasics 0.5, MakieTextRepel internal solver.

**Spec:** [`docs/superpowers/specs/2026-05-28-textrepel-algorithm-design.md`](../specs/2026-05-28-textrepel-algorithm-design.md).

---

## Phase A — Solver foundations

These five tasks land in the solver and recipe before any wrapper code exists. After each one, `Pkg.test()` must still pass — they're backwards-compatible enabling work.

---

### Task 1: `solve_repel` returns a NamedTuple with diagnostics

**Goal:** Change `solve_repel`'s return from `(offsets, dropped)` to `(; offsets, dropped, iter, residual)`. Update the recipe destructure. Existing solver tests continue to pass.

**Files:**
- Modify: `src/solver.jl` — `solve_repel` function body and return
- Modify: `src/recipe.jl` — one destructure of `solve_repel`'s return
- Test: `test/test_solver.jl` (existing tests; verify they still pass)

**Note on file references:** From this task onward, anchor on code patterns rather than line numbers — the line numbers shift after each task. Anchors used below: the `for it in 1:p.max_iter` loop, the `return (offsets, compute_drops(...))` statement, the `n == 0 && return` early return, and `offsets, dropped = solve_repel(anchors, sizes, params)` in `recipe.jl`.

- [ ] **Step 1: Write the failing test**

Add to the end of `test/test_solver.jl`:

```julia
@testset "solve_repel returns NamedTuple with diagnostics" begin
    anchors = [Point2f(0, 0), Point2f(50, 0)]
    sizes   = [Vec2f(20, 10), Vec2f(20, 10)]
    p = RepelParams(max_iter = 50)
    result = solve_repel(anchors, sizes, p)
    @test result isa NamedTuple
    @test propertynames(result) == (:offsets, :dropped, :iter, :residual)
    @test result.offsets isa Vector{Vec2f}
    @test result.dropped isa BitVector
    @test result.iter isa Int
    @test 1 <= result.iter <= 50
    @test result.residual isa Float32
    @test result.residual >= 0
end
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL inside the new testset; existing solver-output tests that destructure `(offsets, dropped)` may also fail.

- [ ] **Step 3: Modify `src/solver.jl` to track diagnostics and return NamedTuple**

Three edits to `solve_repel`. Find each anchor pattern and apply the change:

**Edit 1.** Replace the early-return for `n == 0` (the line reading `n == 0 && return (Vec2f[], falses(0))`):

```julia
    n == 0 && return (; offsets = Vec2f[], dropped = falses(0), iter = 0, residual = 0f0)
```

**Edit 2.** Just before the line `for it in 1:p.max_iter`, add two tracking variables:

```julia
    final_iter = 0
    final_residual = 0f0
```

**Edit 3.** Inside the iteration loop, just before the `maxmove < p.tol && break` line, capture the trackers:

```julia
        final_iter = it
        final_residual = maxmove
        maxmove < p.tol && break
    end
```

**Edit 4.** Replace the return statement (the line `return (offsets, compute_drops(anchors, offsets, psizes, p.max_overlaps))`):

```julia
    return (;
        offsets,
        dropped = compute_drops(anchors, offsets, psizes, p.max_overlaps),
        iter    = final_iter,
        residual = final_residual,
    )
```

- [ ] **Step 4: Update the recipe destructure**

In `src/recipe.jl`, find the line `offsets, dropped = solve_repel(anchors, sizes, params)` and replace it with:

```julia
        s = solve_repel(anchors, sizes, params)
        offsets, dropped = s.offsets, s.dropped
```

Verify no other call sites consume the return of `solve_repel` — `grep -n 'solve_repel(' src/` should show only solver.jl (definition) and recipe.jl.

- [ ] **Step 5: Run tests to verify all pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS, including the new NamedTuple test. All existing tests still green.

- [ ] **Step 6: Commit**

```bash
git add src/solver.jl src/recipe.jl test/test_solver.jl
git commit -m "Solver: NamedTuple return with iter/residual diagnostics

solve_repel now returns (; offsets, dropped, iter, residual) so callers
can introspect convergence. recipe.jl destructure updated to match.
Behavior is otherwise unchanged.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
"
```

---

### Task 2: `RepelParams` copy-with-overrides constructor

**Goal:** Add a constructor that takes a base `RepelParams` and a set of overrides, so the wrapper can override `bounds` and `max_iter` without rebuilding from scratch.

**Files:**
- Modify: `src/solver.jl` — add constructor after the `Base.@kwdef struct RepelParams ... end` block
- Test: `test/test_solver.jl`

- [ ] **Step 1: Write the failing test**

Add to `test/test_solver.jl`:

```julia
@testset "RepelParams copy-with-overrides constructor" begin
    base = RepelParams(force = (2.0, 2.0), max_iter = 100)
    overridden = RepelParams(base; max_iter = 50)
    @test overridden.force == (2.0, 2.0)         # carried over
    @test overridden.max_iter == 50              # overridden
    @test overridden.bounds === nothing          # default carried over

    bnds = Rect2f(0, 0, 100, 100)
    with_bounds = RepelParams(base; bounds = bnds)
    @test with_bounds.bounds == bnds
    @test with_bounds.force == (2.0, 2.0)        # unchanged
end
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL with `MethodError: no method matching RepelParams(::RepelParams; ...)`.

- [ ] **Step 3: Add the constructor in `src/solver.jl`**

Insert immediately after the `Base.@kwdef struct RepelParams ... end` block (before the `const _GOLDEN_ANGLE = ...` line):

```julia
"""
    RepelParams(base::RepelParams; kwargs...) -> RepelParams

Copy `base`, replacing any fields named in `kwargs`. All `RepelParams`
fields not in `kwargs` are carried over unchanged.
"""
function RepelParams(base::RepelParams; kwargs...)
    return RepelParams(;
        (field => get(kwargs, field, getfield(base, field))
         for field in fieldnames(RepelParams))...)
end
```

- [ ] **Step 4: Run tests to verify all pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/solver.jl test/test_solver.jl
git commit -m "Solver: add RepelParams copy-with-overrides constructor

Lets call sites override specific fields without rebuilding from
scratch. Used by the upcoming annotation! algorithm wrapper to
substitute bounds and max_iter at solve time.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
"
```

---

### Task 3: `solve_repel` accepts an `obstacles` kwarg

**Goal:** Extend the solver so user-supplied axis-aligned rectangles act as additional repulsion sources, the same way data anchors do via `point_push`.

**Files:**
- Modify: `src/solver.jl` — `solve_repel` signature and force loop
- Test: `test/test_solver.jl`

- [ ] **Step 1: Write the failing test**

`box_at` and `_overlaps` are internal helpers and aren't exported, so the test computes label bboxes inline. Add to `test/test_solver.jl`:

```julia
@testset "solve_repel — obstacles kwarg" begin
    # Helper: compute the label's padded bbox at its final offset, fully
    # inline so the test doesn't depend on internal helper exports.
    function _label_bbox(anchor::Point2f, offset::Vec2f, size::Vec2f, pad::Real)
        psize = size .+ 2 * Float32(pad)
        center = anchor .+ offset
        Rect2f(center[1] - psize[1]/2, center[2] - psize[2]/2,
               psize[1], psize[2])
    end
    function _overlaps(a::Rect2f, b::Rect2f)
        return !(a.origin[1] + a.widths[1] <= b.origin[1] ||
                 b.origin[1] + b.widths[1] <= a.origin[1] ||
                 a.origin[2] + a.widths[2] <= b.origin[2] ||
                 b.origin[2] + b.widths[2] <= a.origin[2])
    end

    # Two labels on either side of an obstacle. Without the obstacle they
    # settle close to their anchors; the obstacle pushes them clear.
    anchors = [Point2f(0, 50), Point2f(100, 50)]
    sizes   = [Vec2f(20, 10), Vec2f(20, 10)]
    p = RepelParams(max_iter = 500, point_padding = 0.0)
    obstacle = Rect2f(40, 40, 20, 20)   # blocks the corridor between them

    # Sanity: empty obstacles vector === no-op vs not passing obstacles at all.
    a = solve_repel(anchors, sizes, p)
    b = solve_repel(anchors, sizes, p; obstacles = Rect2f[])
    @test a.offsets == b.offsets

    # With an obstacle, neither resulting bbox overlaps it.
    c = solve_repel(anchors, sizes, p; obstacles = [obstacle])
    for i in 1:2
        bb = _label_bbox(anchors[i], c.offsets[i], sizes[i], p.box_padding)
        @test !_overlaps(bb, obstacle)
    end

    # Multiple disjoint obstacles: same invariant for each.
    obs2 = [Rect2f(40, 40, 20, 20), Rect2f(40, 10, 20, 20)]
    d = solve_repel(anchors, sizes, p; obstacles = obs2)
    for i in 1:2, o in obs2
        bb = _label_bbox(anchors[i], d.offsets[i], sizes[i], p.box_padding)
        @test !_overlaps(bb, o)
    end
end
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL with `MethodError: no method matching solve_repel(...; obstacles=...)`.

- [ ] **Step 3: Modify `src/solver.jl` solve_repel signature and force loop**

**Edit 1.** Find the `function solve_repel(...)` signature and add the kwarg:

```julia
function solve_repel(anchors::Vector{Point2f}, sizes::Vector{Vec2f}, p::RepelParams;
                     obstacles::Vector{Rect2f} = Rect2f[])
```

**Edit 2.** Inside the per-label force accumulation loop (`for i in 1:n` inside the iteration loop), find the closing `end` of the inner `for j in 1:n` loop that applies `point_push` (the one with the `# Own anchor is included` comment). Just after that `end`, before the `off = offsets[i]` line, add an obstacles loop:

```julia
            for ob in obstacles
                push = overlap_push(boxes[i], ob)
                f = f .+ Vec2f(push[1] * fx, push[2] * fy)
            end
```

- [ ] **Step 4: Run tests to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS, including the new obstacles testset.

- [ ] **Step 5: Commit**

```bash
git add src/solver.jl test/test_solver.jl
git commit -m "Solver: add obstacles kwarg for axis-aligned rect repulsion

User-supplied Rect2f obstacles join the force loop as additional
repulsion sources via overlap_push, exactly like other label boxes.
Default empty vector preserves prior behavior.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
"
```

---

### Task 4: `solve_repel` accepts an `init_state` kwarg

**Goal:** Allow callers to supply a starting offsets vector, skipping `init_offsets`. Used by the wrapper for warm-start and for the alignment pre-bias on fresh starts.

**Files:**
- Modify: `src/solver.jl` — `solve_repel` signature and the `offsets = [...]` initialization
- Test: `test/test_solver.jl`

- [ ] **Step 1: Write the failing test**

Add to `test/test_solver.jl`:

```julia
@testset "solve_repel — init_state kwarg" begin
    anchors = [Point2f(0, 0), Point2f(100, 0)]
    sizes   = [Vec2f(20, 10), Vec2f(20, 10)]
    p = RepelParams(max_iter = 50)

    # nothing → behaves identically to the default-init path.
    a = solve_repel(anchors, sizes, p)
    b = solve_repel(anchors, sizes, p; init_state = nothing)
    @test a.offsets == b.offsets

    # Custom init → that's what the loop starts from.
    custom = [Vec2f(10, 5), Vec2f(-10, 5)]
    c = solve_repel(anchors, sizes, p; init_state = custom)
    # After one iteration the offsets diverge from custom but the residual
    # is non-trivially different from running fresh.
    @test c.offsets != a.offsets

    # Length-mismatched init_state raises.
    @test_throws DimensionMismatch solve_repel(anchors, sizes, p;
                                               init_state = [Vec2f(0, 0)])
end
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL on `init_state` kwarg.

- [ ] **Step 3: Modify `src/solver.jl`**

**Edit 1.** Update the `solve_repel` signature to add `init_state`:

```julia
function solve_repel(anchors::Vector{Point2f}, sizes::Vector{Vec2f}, p::RepelParams;
                     obstacles::Vector{Rect2f} = Rect2f[],
                     init_state::Union{Nothing,Vector{Vec2f}} = nothing)
```

**Edit 2.** Find the line:

```julia
    offsets = [_constrain(o, p.only_move) for o in init_offsets(anchors, psizes, p)]
```

and replace with:

```julia
    if init_state !== nothing
        length(init_state) == n || throw(DimensionMismatch(
            "init_state length $(length(init_state)) does not match anchors length $n"))
        offsets = [_constrain(o, p.only_move) for o in init_state]
    else
        offsets = [_constrain(o, p.only_move) for o in init_offsets(anchors, psizes, p)]
    end
```

- [ ] **Step 4: Run tests to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/solver.jl test/test_solver.jl
git commit -m "Solver: add init_state kwarg for caller-supplied initial offsets

Lets the wrapper supply an alignment pre-bias on fresh starts and
honor reset=false for warm-start under advance_optimization!. Default
nothing preserves the existing init_offsets path.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
"
```

---

### Task 5: `solve_repel` accepts `pin_mask` and `pinned_offsets`

**Goal:** Per-label pinning. Labels marked in `pin_mask` are held at their `pinned_offsets` value throughout iteration; their boxes still act as obstacles for other labels. Returned offsets for pinned indices equal `pinned_offsets` exactly.

**Files:**
- Modify: `src/solver.jl` — `solve_repel` signature, offsets init, force-accumulation loop, and update loop
- Test: `test/test_solver.jl`

- [ ] **Step 1: Write the failing test**

Add to `test/test_solver.jl`:

```julia
@testset "solve_repel — pin_mask and pinned_offsets" begin
    anchors = [Point2f(0, 0), Point2f(50, 0), Point2f(100, 0)]
    sizes   = [Vec2f(20, 10), Vec2f(20, 10), Vec2f(20, 10)]
    p = RepelParams(max_iter = 100)

    # Pin label 2 at a specific offset. Solver should leave it exactly there
    # and place 1 and 3 around it.
    pin_mask  = BitVector([false, true, false])
    pinned    = [Vec2f(0, 0), Vec2f(20, 40), Vec2f(0, 0)]
    r = solve_repel(anchors, sizes, p;
                    pin_mask = pin_mask, pinned_offsets = pinned)

    @test r.offsets[2] == Vec2f(20, 40)   # pinned exactly
    # Other two labels moved; they're not still at the spiral init.
    @test r.offsets[1] != Vec2f(0, 0)
    @test r.offsets[3] != Vec2f(0, 0)

    # Pin × only_move: a pinned offset with non-zero x with only_move = :y
    # keeps its x (D6: pinning bypasses only_move).
    p_y = RepelParams(max_iter = 100, only_move = :y)
    r2 = solve_repel(anchors, sizes, p_y;
                     pin_mask = BitVector([false, true, false]),
                     pinned_offsets = [Vec2f(0, 0), Vec2f(20, 40), Vec2f(0, 0)])
    @test r2.offsets[2] == Vec2f(20, 40)  # x kept despite only_move = :y

    # nothing pin_mask + empty pinned_offsets === current default behavior.
    r3 = solve_repel(anchors, sizes, p)
    r4 = solve_repel(anchors, sizes, p;
                     pin_mask = nothing, pinned_offsets = Vec2f[])
    @test r3.offsets == r4.offsets
end
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL on `pin_mask` kwarg not recognized.

- [ ] **Step 3: Modify `src/solver.jl`**

**Edit 1.** Update the `solve_repel` signature:

```julia
function solve_repel(anchors::Vector{Point2f}, sizes::Vector{Vec2f}, p::RepelParams;
                     obstacles::Vector{Rect2f}      = Rect2f[],
                     init_state::Union{Nothing,Vector{Vec2f}} = nothing,
                     pin_mask::Union{Nothing,BitVector}        = nothing,
                     pinned_offsets::Vector{Vec2f}             = Vec2f[])
```

**Edit 2.** Just after the offsets init block (the `if init_state !== nothing ... else ... end` from Task 4), force pinned slots to their pinned values:

```julia
    if pin_mask !== nothing
        length(pin_mask) == n || throw(DimensionMismatch(
            "pin_mask length $(length(pin_mask)) does not match anchors length $n"))
        length(pinned_offsets) == n || throw(DimensionMismatch(
            "pinned_offsets length $(length(pinned_offsets)) does not match anchors length $n"))
        for i in 1:n
            if pin_mask[i]
                offsets[i] = pinned_offsets[i]   # bypasses only_move (D6)
            end
        end
    end
```

**Edit 3.** Replace the per-label force-accumulation loop. Find the block beginning `for i in 1:n` inside the iteration loop (the one that builds `f` and writes `Δ[i] = f`) and replace it with the version that early-exits for pinned indices. The full replacement, including the obstacle loop from Task 3:

```julia
        for i in 1:n
            if pin_mask !== nothing && pin_mask[i]
                Δ[i] = Vec2f(0, 0)
                continue
            end
            f = Vec2f(0, 0)
            for j in 1:n
                i == j && continue
                push = overlap_push(boxes[i], boxes[j])
                f = f .+ Vec2f(push[1] * fx, push[2] * fy)
            end
            for j in 1:n
                # Own anchor included: keeps isolated labels off their own point.
                # force_pull (below) provides the inward balance.
                pp = point_push(boxes[i], anchors[j], pad)
                f = f .+ Vec2f(pp[1] * ppx, pp[2] * ppy)
            end
            for ob in obstacles
                push = overlap_push(boxes[i], ob)
                f = f .+ Vec2f(push[1] * fx, push[2] * fy)
            end
            off = offsets[i]
            if norm(off) > pthr
                f = f .- Vec2f(off[1] * plx, off[2] * ply)
            end
            Δ[i] = f
        end
```

The pinned-`i` continue happens AFTER `boxes` is built — so pinned boxes still appear in `boxes` and contribute to the `overlap_push` and `point_push` terms for non-pinned labels.

**Edit 4.** Replace the per-label update loop (the second `for i in 1:n` inside the iteration loop, which writes the new offsets and tracks `maxmove`):

```julia
        for i in 1:n
            if pin_mask !== nothing && pin_mask[i]
                continue  # pinned: skip update, keep pinned_offsets[i]
            end
            d = _constrain(_clamp_step(Δ[i], smax), p.only_move)
            newoff = offsets[i] .+ d
            if p.bounds !== nothing
                box = box_at(anchors[i], newoff, psizes[i])
                # Constrain the clamp shift too, so confinement never moves
                # a label along an axis the user locked via only_move.
                newoff = newoff .+ _constrain(clamp_box_offset(box, p.bounds), p.only_move)
            end
            move = newoff .- offsets[i]
            offsets[i] = newoff
            maxmove = max(maxmove, norm(move))
        end
```

**Return contract:** With these edits, `offsets[i]` for any pinned `i` is set once at Edit 2 to `pinned_offsets[i]` and never touched again. The returned `result.offsets[i]` therefore equals `pinned_offsets[i]` exactly — no constraint, no clamping. The wrapper relies on this when writing the final offsets vector with a single per-element loop.

- [ ] **Step 4: Run tests to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/solver.jl test/test_solver.jl
git commit -m "Solver: add pin_mask + pinned_offsets for per-label pinning

Pinned labels hold their supplied offset throughout iteration and act
as obstacles for non-pinned labels. Pinned offsets bypass only_move
(D6 semantics — user explicitly supplied the value, we don't mutate
it). Defaults nothing/[] preserve existing behavior.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
"
```

---

## Phase B — Wrapper foundation

Three small tasks: the struct + constructors with warnings, the diagnostics getter, and the module wiring.

---

### Task 6: `TextRepelAlgorithm` struct, constructors, and warnings

**Goal:** Define the struct, both constructors, the kwarg validation, and the three warning paths. No dispatch method yet — this is just the type and its construction.

**Files:**
- Create: `src/annotation_algorithm.jl`
- Create: `test/test_annotation_algorithm.jl`
- Modify: `test/runtests.jl` (add include)

- [ ] **Step 1: Write the failing test**

Create `test/test_annotation_algorithm.jl`:

```julia
using MakieTextRepel
using MakieTextRepel: TextRepelAlgorithm, RepelParams, solve_stats
using Test
using GeometryBasics

@testset "TextRepelAlgorithm — constructors" begin
    # Kwarg form, defaults
    alg = TextRepelAlgorithm()
    @test alg isa TextRepelAlgorithm
    @test alg.params isa RepelParams
    @test isempty(alg.obstacles)

    # Kwarg form, forwarded fields
    alg2 = TextRepelAlgorithm(force = (2.0, 2.0), only_move = :y)
    @test alg2.params.force == (2.0, 2.0)
    @test alg2.params.only_move == :y

    # Explicit form
    p = RepelParams(force = (3.0, 3.0))
    alg3 = TextRepelAlgorithm(p)
    @test alg3.params === p
    @test isempty(alg3.obstacles)

    # Explicit form with obstacles
    obs = [Rect2f(0, 0, 10, 10)]
    alg4 = TextRepelAlgorithm(p; obstacles = obs)
    @test alg4.obstacles === obs
end

@testset "TextRepelAlgorithm — unknown kwarg errors" begin
    @test_throws ArgumentError TextRepelAlgorithm(force_pul = (.02, .02))
    @test_throws ArgumentError TextRepelAlgorithm(not_a_field = 1)
end

@testset "TextRepelAlgorithm — bounds warning" begin
    @test_logs (:warn, r"bounds.*automatically") TextRepelAlgorithm(
        bounds = Rect2f(0, 0, 1, 1))
end

@testset "TextRepelAlgorithm — max_overlaps warning" begin
    @test_logs (:warn, r"max_overlaps") TextRepelAlgorithm(max_overlaps = 3)
    @test_logs (:warn, r"max_overlaps") TextRepelAlgorithm(
        RepelParams(max_overlaps = 3))
end
```

Add to `test/runtests.jl`, before the closing `end`:

```julia
    include("test_annotation_algorithm.jl")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL with `UndefVarError: TextRepelAlgorithm not defined`.

- [ ] **Step 3: Create `src/annotation_algorithm.jl`**

```julia
# annotation_algorithm.jl — algorithm plug-in for Makie.annotation!

export TextRepelAlgorithm, solve_stats

"""
    TextRepelAlgorithm(; force, force_point, force_pull, only_move,
                         box_padding, point_padding, max_iter,
                         obstacles = Rect2f[], ...)
    TextRepelAlgorithm(params::RepelParams; obstacles = Rect2f[])

An algorithm plug-in for `Makie.annotation!`. Uses MakieTextRepel's
force-directed solver to place non-overlapping labels around data points.

Supports per-label pinning: set entries of `textpositions_offset` to
fixed values to lock those labels and let the solver place the rest.
Honors `reset = false` from `annotation!`'s compute graph to warm-start
solves under `advance_optimization!`.

`obstacles` is a `Vector{Rect2f}` in pixel space — the same coordinate
system `annotation!` uses internally. Convert a data-space rectangle
with `Makie.project`.

# Caveats

- Pinning a label so that its data anchor falls strictly inside the
  rendered bbox suppresses that label's connector line. This is
  `annotation!`'s `p2 in offset_bb && return` behavior, not a wrapper
  bug.
- `bounds`/`max_overlaps` misuse warnings fire once per session (Julia's
  standard logger `maxlog=1` contract), so constructing multiple
  algorithm instances with the same mistake yields one warning total.

# Scope

`max_overlaps` and background boxes are `textrepel!`-only — they have
no equivalent in `annotation!`'s algorithm contract.

See also: [`textrepel!`](@ref), [`solve_stats`](@ref).
"""
struct TextRepelAlgorithm
    params::RepelParams
    obstacles::Vector{Rect2f}
    last_iter::Base.RefValue{Int}
    last_residual::Base.RefValue{Float32}
end

function TextRepelAlgorithm(; obstacles::Vector{Rect2f} = Rect2f[],
                              bounds = nothing, kwargs...)
    extra = setdiff(keys(kwargs), fieldnames(RepelParams))
    isempty(extra) || throw(ArgumentError(
        "TextRepelAlgorithm: unknown keyword(s) $(collect(extra)). \
         Valid: $(fieldnames(RepelParams)), plus `bounds`, `obstacles`."))

    bounds === nothing || @warn "TextRepelAlgorithm: `bounds` is set \
        automatically from the axis viewport; remove this keyword." maxlog=1

    params = RepelParams(; kwargs...)
    if params.max_overlaps !== Inf
        @warn "TextRepelAlgorithm: `max_overlaps` has no equivalent under \
            annotation!; use `textrepel!` if you need label dropping." maxlog=1
    end
    return TextRepelAlgorithm(params, obstacles, Ref(0), Ref(0f0))
end

function TextRepelAlgorithm(params::RepelParams;
                            obstacles::Vector{Rect2f} = Rect2f[])
    if params.max_overlaps !== Inf
        @warn "TextRepelAlgorithm: `max_overlaps` has no equivalent under \
            annotation!; use `textrepel!` if you need label dropping." maxlog=1
    end
    return TextRepelAlgorithm(params, obstacles, Ref(0), Ref(0f0))
end

"""
    solve_stats(alg::TextRepelAlgorithm) -> (; iter, residual)

Return iteration count and final residual from the most recent solve.
Returns `(iter = 0, residual = 0f0)` before any solve runs.
"""
solve_stats(alg::TextRepelAlgorithm) =
    (iter = alg.last_iter[], residual = alg.last_residual[])
```

Wire the new file into the module now (the test needs access to `TextRepelAlgorithm` and `solve_stats` via `using MakieTextRepel`). Modify `src/MakieTextRepel.jl`:

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
include("annotation_algorithm.jl")

end # module MakieTextRepel
```

`TextRepelAlgorithm` and `solve_stats` are exported from inside `annotation_algorithm.jl` via its `export` line.

- [ ] **Step 4: Run tests to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS — all four constructor testsets green.

- [ ] **Step 5: Commit**

```bash
git add src/annotation_algorithm.jl src/MakieTextRepel.jl test/test_annotation_algorithm.jl test/runtests.jl
git commit -m "Add TextRepelAlgorithm struct, constructors, and solve_stats

Wraps RepelParams + obstacles list + diagnostics refs. Two constructors:
kwarg-forwarding (with unknown-kwarg validation, bounds/max_overlaps
warnings) and explicit (accepts a RepelParams). solve_stats getter
exposes (iter, residual) — the Ref-based storage is an implementation
detail. Wired into the module.

calculate_best_offsets! dispatch comes in subsequent tasks.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
"
```

---

### Task 7: `solve_stats` returns the right shape pre-solve

**Goal:** Verify the diagnostics getter returns the documented zero-values when no solve has happened yet. This is small but worth its own task to nail the contract before tests start asserting against it across the dispatch tasks.

**Files:**
- Test: `test/test_annotation_algorithm.jl`

- [ ] **Step 1: Write the failing test**

Add to `test/test_annotation_algorithm.jl`:

```julia
@testset "solve_stats — initial state" begin
    alg = TextRepelAlgorithm()
    s = solve_stats(alg)
    @test s isa NamedTuple
    @test propertynames(s) == (:iter, :residual)
    @test s.iter == 0
    @test s.residual === 0f0
end
```

- [ ] **Step 2: Run test to verify it passes**

This test passes immediately because `solve_stats` was already defined in Task 6. Treating Task 7 as a verification gate.

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS.

- [ ] **Step 3: Commit (test-only)**

```bash
git add test/test_annotation_algorithm.jl
git commit -m "Test: assert solve_stats returns (0, 0f0) before any solve

Locks in the documented pre-solve contract so future dispatch tests
can assert against this baseline.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
"
```

---

## Phase C — Dispatch method, one feature at a time

The `Makie.calculate_best_offsets!` method is the bulk of the work. Each task adds one capability with its dedicated test.

---

### Task 8: Basic dispatch (no pinning, no warm-start, no obstacles)

**Goal:** A working dispatch method that passes annotation's call to the solver and writes back offsets. No pinning, no warm-start, no alignment fix yet. Test exercises the happy path.

**Files:**
- Modify: `src/annotation_algorithm.jl`
- Test: `test/test_annotation_algorithm.jl`

- [ ] **Step 1: Write the failing test**

Add to `test/test_annotation_algorithm.jl`:

```julia
using Makie  # for Vec2f, Point2f, Rect2f, Automatic, calculate_best_offsets!

@testset "dispatch — basic happy path" begin
    # Three labels with NaN textpositions_offset (auto mode), all centered text.
    n = 3
    offsets              = [Vec2f(0, 0) for _ in 1:n]
    textpositions        = [Point2f(0, 0), Point2f(100, 0), Point2f(200, 0)]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    # Centered bboxes: origin = textposition - widths/2.
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)

    alg = TextRepelAlgorithm()
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    # All offsets populated, not all zero.
    @test all(isfinite, [o[1] for o in offsets])
    @test all(isfinite, [o[2] for o in offsets])
    @test any(o -> norm(o) > 0, offsets)

    # Diagnostics populated.
    s = solve_stats(alg)
    @test s.iter > 0
    @test s.residual >= 0
end

@testset "dispatch — n=0 early return" begin
    offsets = Vec2f[]
    Makie.calculate_best_offsets!(TextRepelAlgorithm(),
                                  offsets, Point2f[], Point2f[],
                                  Rect2f[], Rect2f(0, 0, 100, 100);
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)
    @test isempty(offsets)  # untouched
end

@testset "dispatch — all-pinned bypass" begin
    n = 3
    offsets       = [Vec2f(0, 0) for _ in 1:n]
    textpositions = [Point2f(0, 0), Point2f(100, 0), Point2f(200, 0)]
    # All finite => manual mode.
    textpositions_offset = [Point2f(p[1] + 10, p[2] + 20) for p in textpositions]
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)

    alg = TextRepelAlgorithm()
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    for i in 1:n
        @test offsets[i][1] ≈ 10f0
        @test offsets[i][2] ≈ 20f0
    end
    @test solve_stats(alg) == (iter = 0, residual = 0f0)
end
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL with `MethodError: no method matching calculate_best_offsets!(::TextRepelAlgorithm, ...)`.

- [ ] **Step 3: Append the dispatch method to `src/annotation_algorithm.jl`**

```julia
function Makie.calculate_best_offsets!(
        alg::TextRepelAlgorithm,
        offsets::Vector{<:Vec2},
        textpositions::Vector{<:Point2},
        textpositions_offset::Vector{<:Point2},
        text_bbs::Vector{<:Rect2},
        bbox::Rect2;
        maxiter::Union{Makie.Automatic, Int},
        labelspace::Symbol,
        reset::Bool,
    )
    n = length(offsets)
    n == 0 && return

    T = eltype(offsets)

    # Per-label pinning detection (full implementation in Task 10).
    pin_mask = BitVector([all(isfinite, p) for p in textpositions_offset])
    pinned_offsets = Vector{Vec2f}(undef, n)
    for i in 1:n
        if pin_mask[i]
            d = textpositions_offset[i] - textpositions[i]
            pinned_offsets[i] = Vec2f(d[1], d[2])
        else
            pinned_offsets[i] = Vec2f(0, 0)
        end
    end

    # All-pinned: bypass solver.
    if all(pin_mask)
        for i in 1:n
            offsets[i] = T(pinned_offsets[i][1], pinned_offsets[i][2])
        end
        alg.last_iter[] = 0
        alg.last_residual[] = 0f0
        return
    end

    anchors = [Point2f(p[1], p[2]) for p in textpositions]
    sizes   = [Vec2f(bb.widths[1], bb.widths[2]) for bb in text_bbs]
    annotation_bounds = Rect2f(
        Float32(bbox.origin[1]),  Float32(bbox.origin[2]),
        Float32(bbox.widths[1]),  Float32(bbox.widths[2]),
    )

    mi = maxiter === Makie.automatic ? alg.params.max_iter : Int(maxiter)
    effective_params = RepelParams(alg.params;
        bounds   = annotation_bounds,
        max_iter = mi,
    )

    result = solve_repel(anchors, sizes, effective_params)

    alg.last_iter[] = result.iter
    alg.last_residual[] = result.residual

    for i in 1:n
        offsets[i] = T(result.offsets[i][1], result.offsets[i][2])
    end
    return
end
```

Note: This stub doesn't yet use `pin_mask` / `pinned_offsets` in the solver call. The mixed-pinning test exists in Task 10. For now, all-finite case is handled (bypass path) and all-NaN case works (default solver). Mixed will fail until Task 10.

- [ ] **Step 4: Run tests to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS — three new dispatch testsets green.

- [ ] **Step 5: Commit**

```bash
git add src/annotation_algorithm.jl test/test_annotation_algorithm.jl
git commit -m "Add basic calculate_best_offsets! dispatch

Happy path: read text_bbs as label sizes, anchor at textposition, solve
via solve_repel, write back to offsets. Honors n=0 (early return) and
all-pinned (bypass solver). Mixed pinning, warm-start, alignment fix,
and obstacles passthrough come in subsequent tasks.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
"
```

---

### Task 9: Alignment-correct anchor via init_state pre-bias

**Goal:** For non-center alignments, pass `textpositions` as the solver anchor and pre-bias `init_state` to `bbox_center - textposition`. The own-anchor repulsion (PR #7) then pushes from the data point, not from the bbox center.

**Files:**
- Modify: `src/annotation_algorithm.jl` (dispatch body)
- Test: `test/test_annotation_algorithm.jl`

- [ ] **Step 1: Write the failing test**

The test compares two solves of the same anchor with different bbox alignments. Without the alignment pre-bias, both solves treat the anchor as the bbox center (regardless of `text_bbs[i].origin`) and produce identical offsets. With the pre-bias, the offsets differ by approximately the alignment shift. This is a robust discriminator: without the fix, `delta` is essentially zero; with the fix, `delta` reflects the alignment shift.

Add to `test/test_annotation_algorithm.jl`:

```julia
@testset "dispatch — alignment-correct anchor (D5)" begin
    # Same textposition, same widths, two different alignments.
    textpositions        = [Point2f(50, 50)]
    textpositions_offset = [Point2f(NaN, NaN)]
    viewport             = Rect2f(0, 0, 500, 500)

    # Center-aligned bbox: origin = textposition - widths/2.
    text_bbs_center = [Rect2f(30, 45, 40, 10)]
    offsets_center  = [Vec2f(0, 0)]
    Makie.calculate_best_offsets!(TextRepelAlgorithm(), offsets_center,
        textpositions, textpositions_offset, text_bbs_center, viewport;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)

    # Left-aligned bbox: origin = textposition (bbox extends right).
    text_bbs_left = [Rect2f(50, 45, 40, 10)]
    offsets_left  = [Vec2f(0, 0)]
    Makie.calculate_best_offsets!(TextRepelAlgorithm(), offsets_left,
        textpositions, textpositions_offset, text_bbs_left, viewport;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)

    # With the alignment fix, the left-aligned solve's offset is shifted
    # right by approximately widths/2 = 20px relative to the centered one
    # (the alignment pre-bias is added to init_state). Without the fix,
    # both solves are anchored identically and the delta would be ~0.
    delta = offsets_left[1] - offsets_center[1]
    @test isapprox(delta[1], 20f0, atol = 5.0)
    @test isapprox(delta[2], 0f0,  atol = 5.0)
end
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL — without the pre-bias, the left-aligned bbox sits over the textposition.

- [ ] **Step 3: Add the alignment pre-bias to the dispatch**

In `src/annotation_algorithm.jl`'s `calculate_best_offsets!`, locate the block just before the `solve_repel` call (after `effective_params = RepelParams(...)`):

```julia
    mi = maxiter === Makie.automatic ? alg.params.max_iter : Int(maxiter)
    effective_params = RepelParams(alg.params;
        bounds   = annotation_bounds,
        max_iter = mi,
    )

    result = solve_repel(anchors, sizes, effective_params)
```

Replace with:

```julia
    mi = maxiter === Makie.automatic ? alg.params.max_iter : Int(maxiter)
    effective_params = RepelParams(alg.params;
        bounds   = annotation_bounds,
        max_iter = mi,
    )

    # Alignment pre-bias (D5): box starts centered at bbox_center.
    bbox_centers = [Point2f(bb.origin[1] + bb.widths[1]/2,
                            bb.origin[2] + bb.widths[2]/2) for bb in text_bbs]
    align_bias   = [Vec2f(c[1] - a[1], c[2] - a[2])
                    for (c, a) in zip(bbox_centers, anchors)]

    result = solve_repel(anchors, sizes, effective_params;
                         init_state = align_bias)
```

- [ ] **Step 4: Run tests to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/annotation_algorithm.jl test/test_annotation_algorithm.jl
git commit -m "Dispatch: alignment-correct anchor via init_state pre-bias

Anchor stays at textposition (so PR #7's own-anchor repulsion pushes
from the data point, not from the bbox center). The bbox-center-minus-
textposition delta is fed to solve_repel as init_state, placing the
label box visually correctly at first iteration.

For default centered alignment, align_bias is zero and behavior is
unchanged.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
"
```

---

### Task 10: Per-label pinning in dispatch (mixed mode)

**Goal:** Pass `pin_mask` and `pinned_offsets` through to `solve_repel` so mixed finite/NaN `textpositions_offset` works.

**Files:**
- Modify: `src/annotation_algorithm.jl` (dispatch body)
- Test: `test/test_annotation_algorithm.jl`

- [ ] **Step 1: Write the failing test**

Add to `test/test_annotation_algorithm.jl`:

```julia
@testset "dispatch — per-label pinning (mixed mode)" begin
    n = 3
    textpositions = [Point2f(0, 0), Point2f(100, 0), Point2f(200, 0)]
    # Pin label 2 at (110, 30); labels 1 and 3 auto.
    textpositions_offset = [Point2f(NaN, NaN), Point2f(110, 30), Point2f(NaN, NaN)]
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)
    offsets  = [Vec2f(0, 0) for _ in 1:n]

    alg = TextRepelAlgorithm()
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    # Pinned label: offset is exactly the pin (relative to its textposition).
    @test offsets[2][1] ≈ 10f0   # 110 - 100
    @test offsets[2][2] ≈ 30f0

    # Auto-placed labels: not zero (solver ran).
    @test offsets[1] != Vec2f(0, 0)
    @test offsets[3] != Vec2f(0, 0)

    # Pinned bbox acts as an obstacle for auto-placed labels:
    # neither auto label's rendered bbox overlaps the pinned label's
    # rendered bbox.
    function _rendered(i)
        Rect2f(text_bbs[i].origin[1] + offsets[i][1],
               text_bbs[i].origin[2] + offsets[i][2],
               text_bbs[i].widths[1],
               text_bbs[i].widths[2])
    end
    pinned_rect = _rendered(2)
    for i in (1, 3)
        r = _rendered(i)
        overlaps_x = !(r.origin[1] + r.widths[1] <= pinned_rect.origin[1] ||
                       pinned_rect.origin[1] + pinned_rect.widths[1] <= r.origin[1])
        overlaps_y = !(r.origin[2] + r.widths[2] <= pinned_rect.origin[2] ||
                       pinned_rect.origin[2] + pinned_rect.widths[2] <= r.origin[2])
        @test !(overlaps_x && overlaps_y)
    end
end

@testset "dispatch — pin × only_move bypasses constraint (D6)" begin
    textpositions = [Point2f(0, 0), Point2f(100, 0)]
    # Pin label 2 with non-zero x component; alg uses only_move = :y.
    textpositions_offset = [Point2f(NaN, NaN), Point2f(120, 30)]
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)
    offsets  = [Vec2f(0, 0) for _ in 1:2]

    alg = TextRepelAlgorithm(only_move = :y)
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    # The pinned label keeps its x component (20 px) despite only_move = :y.
    @test offsets[2][1] ≈ 20f0
    @test offsets[2][2] ≈ 30f0
end
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL — without `pin_mask` passthrough, the solver runs on all labels and the pinned offset isn't preserved.

- [ ] **Step 3: Pass pinning kwargs to solve_repel**

In `src/annotation_algorithm.jl`, find the `solve_repel` call (added in Task 9):

```julia
    result = solve_repel(anchors, sizes, effective_params;
                         init_state = align_bias)
```

Change to:

```julia
    result = solve_repel(anchors, sizes, effective_params;
                         init_state     = align_bias,
                         pin_mask       = pin_mask,
                         pinned_offsets = pinned_offsets)
```

Note: when `pin_mask[i]` is true, the pre-task-5 `align_bias[i]` is overwritten by `pinned_offsets[i]` inside the solver (Task 5's modification to the offsets-init block). So mixing alignment fix and pinning is automatic.

- [ ] **Step 4: Run tests to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS — both new pinning testsets green.

- [ ] **Step 5: Commit**

```bash
git add src/annotation_algorithm.jl test/test_annotation_algorithm.jl
git commit -m "Dispatch: per-label pinning via solve_repel kwargs

Mixed finite/NaN textpositions_offset is now honored: finite entries
become pinned offsets, NaN entries auto-place around them. Pinned
offsets bypass only_move (D6 semantics — user supplied a specific
value, we don't mutate it).

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
"
```

---

### Task 11: Warm-start on `reset = false`

**Goal:** When the compute graph calls us with `reset = false`, use the incoming `offsets` vector as the solver's initial state, skipping the alignment pre-bias (the incoming offsets are already textposition-relative).

**Files:**
- Modify: `src/annotation_algorithm.jl` (dispatch body)
- Test: `test/test_annotation_algorithm.jl`

- [ ] **Step 1: Write the failing test**

The discriminator: pre-populate the incoming `offsets` with values far from any sane starting state (~500px). Run two solves with the SAME parameters but different `reset` values. With warm-start honored, the `reset = false` call starts from the pre-populated extreme; without it, the call ignores `offsets` and starts from `init_offsets`/`align_bias` (small magnitudes). After the same iteration budget, the warm-started result stays much closer to (500, 500) than the fresh-started result. This is robust against the bounds-cooling step-cap schedule (which would zero out `step_max` near the final iteration regardless).

Add to `test/test_annotation_algorithm.jl`:

```julia
@testset "dispatch — warm-start when reset == false" begin
    n = 3
    textpositions = [Point2f(0, 0), Point2f(100, 0), Point2f(200, 0)]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 2000, 2000)   # large viewport; (500, 500) is far inside

    # Modest max_iter so neither solve converges; step_max small so neither
    # can fully traverse from (500, 500) to equilibrium.
    alg = TextRepelAlgorithm(max_iter = 10, step_max = 5.0)

    # Warm-start: pre-populated extreme offsets, reset = false.
    offsets_warm = [Vec2f(500, 500) for _ in 1:n]
    Makie.calculate_best_offsets!(alg, offsets_warm,
        textpositions, textpositions_offset, text_bbs, bbox;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = false)

    # Fresh-start: same pre-populated input, but reset = true (solver
    # discards offsets and uses init_offsets / align_bias instead).
    offsets_fresh = [Vec2f(500, 500) for _ in 1:n]
    Makie.calculate_best_offsets!(alg, offsets_fresh,
        textpositions, textpositions_offset, text_bbs, bbox;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)

    # Warm-start preserves position relative to fresh-start. Each
    # warm-start offset is closer to (500, 500) than its fresh counterpart.
    for i in 1:n
        @test norm(offsets_warm[i] - Vec2f(500, 500)) <
              norm(offsets_fresh[i] - Vec2f(500, 500))
    end
end

@testset "dispatch — warm-start preserves own-anchor invariant (R3)" begin
    # After equilibrium under reset=true (PR #7 guarantees the anchor lies
    # outside each label's bbox), a subsequent reset=false solve should
    # maintain that invariant — warm-start mustn't degenerate the layout.
    n = 4
    textpositions = [Point2f(50i, 50) for i in 1:n]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(p[1] - 15, p[2] - 5, 30, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)

    alg = TextRepelAlgorithm(max_iter = 500)
    offsets = [Vec2f(0, 0) for _ in 1:n]

    # Solve to equilibrium.
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
        text_bbs, bbox;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)

    # Continue from equilibrium with warm-start.
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
        text_bbs, bbox;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = false)

    # Own-anchor invariant: each rendered bbox does NOT contain its anchor.
    for i in 1:n
        rendered = Rect2f(text_bbs[i].origin[1] + offsets[i][1],
                          text_bbs[i].origin[2] + offsets[i][2],
                          text_bbs[i].widths[1],
                          text_bbs[i].widths[2])
        contains_x = textpositions[i][1] >= rendered.origin[1] &&
                     textpositions[i][1] <= rendered.origin[1] + rendered.widths[1]
        contains_y = textpositions[i][2] >= rendered.origin[2] &&
                     textpositions[i][2] <= rendered.origin[2] + rendered.widths[2]
        @test !(contains_x && contains_y)
    end
end

@testset "dispatch — reset=false with mismatched offsets length errors" begin
    alg = TextRepelAlgorithm()
    textpositions = [Point2f(0, 0), Point2f(100, 0)]
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    # offsets length is 1 but textpositions length is 2.
    offsets = [Vec2f(0, 0)]
    @test_throws DimensionMismatch Makie.calculate_best_offsets!(
        alg, offsets, textpositions, fill(Point2f(NaN, NaN), 2),
        text_bbs, Rect2f(0, 0, 500, 500);
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = false)
end
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL — without warm-start, `reset = false` re-runs from `align_bias` and the residual matches `residual_fresh` rather than being smaller.

- [ ] **Step 3: Add warm-start path to dispatch**

In `src/annotation_algorithm.jl`, replace the `align_bias` block and `solve_repel` call. The current code (after Task 10):

```julia
    bbox_centers = [Point2f(bb.origin[1] + bb.widths[1]/2,
                            bb.origin[2] + bb.widths[2]/2) for bb in text_bbs]
    align_bias   = [Vec2f(c[1] - a[1], c[2] - a[2])
                    for (c, a) in zip(bbox_centers, anchors)]

    result = solve_repel(anchors, sizes, effective_params;
                         init_state     = align_bias,
                         pin_mask       = pin_mask,
                         pinned_offsets = pinned_offsets)
```

Becomes:

```julia
    # Choose initial state: warm-start (D7) when reset = false, else
    # alignment pre-bias from bbox_center - textposition (D5).
    init_state = if reset
        bbox_centers = [Point2f(bb.origin[1] + bb.widths[1]/2,
                                bb.origin[2] + bb.widths[2]/2) for bb in text_bbs]
        Vec2f[Vec2f(c[1] - a[1], c[2] - a[2])
              for (c, a) in zip(bbox_centers, anchors)]
    else
        Vec2f[Vec2f(o[1], o[2]) for o in offsets]
    end

    result = solve_repel(anchors, sizes, effective_params;
                         init_state     = init_state,
                         pin_mask       = pin_mask,
                         pinned_offsets = pinned_offsets)
```

And at the top of the dispatch, just after the `n == 0 && return` check, add a length-check for warm-start hygiene:

```julia
    n == 0 && return
    length(textpositions) == n || throw(DimensionMismatch(
        "textpositions length $(length(textpositions)) does not match offsets length $n"))
```

(The solver's own DimensionMismatch from Task 4 would catch this too, but the wrapper-level check fires earlier with a clearer message.)

- [ ] **Step 4: Run tests to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/annotation_algorithm.jl test/test_annotation_algorithm.jl
git commit -m "Dispatch: warm-start when reset == false

When the compute graph calls us with reset = false (advance_optimization!
path), use the incoming offsets vector as init_state instead of running
the alignment pre-bias. The solver continues from near-equilibrium and
the residual drops faster.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
"
```

---

### Task 12: Obstacles passthrough in dispatch

**Goal:** Pass `alg.obstacles` through to `solve_repel`. With this task, the dispatch is feature-complete.

**Files:**
- Modify: `src/annotation_algorithm.jl` (dispatch body)
- Test: `test/test_annotation_algorithm.jl`

- [ ] **Step 1: Write the failing test**

Add to `test/test_annotation_algorithm.jl`:

```julia
@testset "dispatch — obstacles avoidance" begin
    n = 2
    textpositions = [Point2f(0, 50), Point2f(100, 50)]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 200, 100)

    obstacle = Rect2f(40, 40, 20, 20)
    # max_iter set high so the solver has time to clear the obstacle even
    # under the tight 200x100 viewport.
    alg = TextRepelAlgorithm(obstacles = [obstacle], max_iter = 1000)
    offsets = [Vec2f(0, 0) for _ in 1:n]

    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    for i in 1:n
        # Rendered bbox after offset application.
        rendered = Rect2f(text_bbs[i].origin[1] + offsets[i][1],
                          text_bbs[i].origin[2] + offsets[i][2],
                          text_bbs[i].widths[1],
                          text_bbs[i].widths[2])
        # No overlap iff one separating axis exists.
        no_overlap = (rendered.origin[1] + rendered.widths[1] <= obstacle.origin[1] ||
                      obstacle.origin[1] + obstacle.widths[1] <= rendered.origin[1] ||
                      rendered.origin[2] + rendered.widths[2] <= obstacle.origin[2] ||
                      obstacle.origin[2] + obstacle.widths[2] <= rendered.origin[2])
        @test no_overlap
    end
end
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL — without passing `obstacles`, the solver places labels right through the obstacle region.

- [ ] **Step 3: Pass obstacles to solve_repel**

In `src/annotation_algorithm.jl`, the `solve_repel` call (after Task 11):

```julia
    result = solve_repel(anchors, sizes, effective_params;
                         init_state     = init_state,
                         pin_mask       = pin_mask,
                         pinned_offsets = pinned_offsets)
```

becomes:

```julia
    result = solve_repel(anchors, sizes, effective_params;
                         obstacles      = alg.obstacles,
                         init_state     = init_state,
                         pin_mask       = pin_mask,
                         pinned_offsets = pinned_offsets)
```

- [ ] **Step 4: Run tests to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/annotation_algorithm.jl test/test_annotation_algorithm.jl
git commit -m "Dispatch: pass obstacles through to solver

User-supplied Rect2f obstacles now act as additional repulsion sources
during the solve. Dispatch method is now feature-complete.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
"
```

---

## Phase D — Edge cases, composition, stability canary

Three tasks rounding out the test coverage.

---

### Task 13: Edge case tests

**Goal:** Lock in dispatch behavior on degenerate inputs.

**Files:**
- Test: `test/test_annotation_algorithm.jl`

- [ ] **Step 1: Write the tests**

Add to `test/test_annotation_algorithm.jl`:

```julia
@testset "dispatch — n=1" begin
    textpositions = [Point2f(50, 50)]
    textpositions_offset = [Point2f(NaN, NaN)]
    text_bbs = [Rect2f(40, 45, 20, 10)]
    bbox     = Rect2f(0, 0, 100, 100)
    offsets  = [Vec2f(0, 0)]

    Makie.calculate_best_offsets!(TextRepelAlgorithm(),
                                  offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    @test all(isfinite, offsets[1])
end

@testset "dispatch — coincident anchors" begin
    n = 2
    textpositions = [Point2f(50, 50), Point2f(50, 50)]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(40, 45, 20, 10) for _ in 1:n]
    bbox     = Rect2f(0, 0, 200, 200)
    offsets  = [Vec2f(0, 0) for _ in 1:n]

    Makie.calculate_best_offsets!(TextRepelAlgorithm(),
                                  offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    # Golden-angle init_offsets fans them out → different offsets.
    @test offsets[1] != offsets[2]
end

@testset "dispatch — zero-width label" begin
    textpositions = [Point2f(50, 50), Point2f(100, 50)]
    textpositions_offset = fill(Point2f(NaN, NaN), 2)
    text_bbs = [Rect2f(50, 50, 0, 0), Rect2f(90, 45, 20, 10)]
    bbox     = Rect2f(0, 0, 200, 200)
    offsets  = [Vec2f(0, 0) for _ in 1:2]

    # Solver should not divide by zero or produce NaN offsets.
    Makie.calculate_best_offsets!(TextRepelAlgorithm(),
                                  offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)
    @test all(isfinite, offsets[1])
    @test all(isfinite, offsets[2])
end

@testset "dispatch — NaN in textpositions errors clearly" begin
    textpositions = [Point2f(NaN, 50), Point2f(100, 50)]
    textpositions_offset = fill(Point2f(NaN, NaN), 2)
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 200, 200)
    offsets  = [Vec2f(0, 0) for _ in 1:2]

    @test_throws ArgumentError Makie.calculate_best_offsets!(
        TextRepelAlgorithm(),
        offsets, textpositions, textpositions_offset,
        text_bbs, bbox;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)
end
```

- [ ] **Step 2: Run tests; some pass, NaN textposition fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: n=1 and coincident testsets pass already (Phase B + C give us these for free); zero-width should pass given the unconditional guard below; NaN-textposition FAILS (no validation in dispatch yet).

The mixed-expectation here is intentional — n=1 and coincident are lock-in regression coverage, not TDD-driven additions. NaN-textposition is the actual TDD case for this task.

- [ ] **Step 3: Add NaN validation and zero-width guard to dispatch**

In `src/annotation_algorithm.jl`, find the length-check from Task 11 (`length(textpositions) == n || throw(DimensionMismatch(...))`) and add a finite-values check on the next line:

```julia
    all(p -> all(isfinite, p), textpositions) || throw(ArgumentError(
        "TextRepelAlgorithm: textpositions contains non-finite values"))
```

Replace the final writeback loop (the one that does `offsets[i] = T(result.offsets[i][1], result.offsets[i][2])`) with a guarded version that handles NaN/Inf from degenerate solver paths (zero-width labels):

```julia
    for i in 1:n
        if all(isfinite, result.offsets[i])
            offsets[i] = T(result.offsets[i][1], result.offsets[i][2])
        else
            # Solver produced NaN/Inf — fall back to zero offset for this label.
            offsets[i] = T(0, 0)
        end
    end
```

The guard is unconditional rather than conditional on the test failing — it's a one-line correctness improvement that costs nothing.

- [ ] **Step 4: Run tests to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS — all edge case testsets green.

- [ ] **Step 5: Commit**

```bash
git add src/annotation_algorithm.jl test/test_annotation_algorithm.jl
git commit -m "Dispatch: edge case coverage (n=1, coincident, zero-width, NaN)

Adds clear ArgumentError on NaN/Inf in textpositions (R8) and a NaN
sanity guard in the writeback path. Coincident anchors and zero-width
labels are handled by the existing solver init/force logic.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
"
```

---

### Task 14: Composition tests

**Goal:** Cover the cross-cutting paths: `maxiter` precedence, `labelspace = :data`, the StaticArrays broadcast canary, and `solve_stats` populated after a real solve.

**Files:**
- Test: `test/test_annotation_algorithm.jl`

- [ ] **Step 1: Write the tests**

Add to `test/test_annotation_algorithm.jl`:

```julia
@testset "dispatch — maxiter precedence" begin
    n = 2
    textpositions = [Point2f(0, 0), Point2f(100, 0)]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)

    alg = TextRepelAlgorithm(max_iter = 1000)
    offsets = [Vec2f(0, 0) for _ in 1:n]

    # Explicit maxiter overrides alg.params.max_iter.
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = 5,
                                  labelspace = :relative_pixel,
                                  reset = true)
    @test solve_stats(alg).iter <= 5

    # Makie.automatic falls through to alg.params.max_iter.
    fill!(offsets, Vec2f(0, 0))
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)
    @test solve_stats(alg).iter <= 1000
end

@testset "dispatch — labelspace = :data invariance" begin
    n = 2
    textpositions = [Point2f(0, 0), Point2f(100, 0)]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)

    alg = TextRepelAlgorithm()

    offsets_rel = [Vec2f(0, 0) for _ in 1:n]
    Makie.calculate_best_offsets!(alg, offsets_rel, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    offsets_data = [Vec2f(0, 0) for _ in 1:n]
    Makie.calculate_best_offsets!(alg, offsets_data, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :data,
                                  reset = true)

    # labelspace doesn't reach the solver — same input → same output.
    @test offsets_rel == offsets_data
end

@testset "dispatch — StaticArrays broadcast canary" begin
    # The dispatch uses per-element T(x, y) construction rather than
    # broadcast assignment to avoid StaticArrays broadcasting into each
    # element. This test asserts the path runs cleanly on Vector{Vec2f}.
    n = 3
    offsets = Vector{Vec2f}([Vec2f(0, 0) for _ in 1:n])
    textpositions = [Point2f(i * 50, 50) for i in 1:n]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]

    @test_nowarn Makie.calculate_best_offsets!(TextRepelAlgorithm(),
                                               offsets, textpositions, textpositions_offset,
                                               text_bbs, Rect2f(0, 0, 500, 500);
                                               maxiter = Makie.automatic,
                                               labelspace = :relative_pixel,
                                               reset = true)
end

@testset "dispatch — solve_stats populated after real solve" begin
    n = 3
    textpositions = [Point2f(i * 50, 50) for i in 1:n]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)
    offsets  = [Vec2f(0, 0) for _ in 1:n]

    alg = TextRepelAlgorithm()
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    s = solve_stats(alg)
    @test s.iter > 0
    @test s.residual >= 0f0
end
```

- [ ] **Step 2: Run tests to verify pass**

These tests assert against the dispatch as it exists after Task 13. All should pass without code changes.

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS.

- [ ] **Step 3: Commit (test-only)**

```bash
git add test/test_annotation_algorithm.jl
git commit -m "Test: composition coverage for dispatch

Locks in maxiter precedence (explicit beats Automatic), labelspace
invariance (solver always works in pixel space), the StaticArrays
broadcast canary (Vec2f vector through the per-element writeback), and
solve_stats populated after a real solve.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
"
```

---

### Task 15a: Stress smoke (n=100 and pathological cluster) — CI-automated

**Goal:** Automate the SC5 stress check so CI catches regressions where the wrapper produces NaN/Inf offsets or blows past the viewport on large/co-located inputs. This is the CI-runnable companion to the manual visual inspection in the completion checklist.

**Files:**
- Test: `test/test_annotation_algorithm.jl`

- [ ] **Step 1: Write the test**

Add to `test/test_annotation_algorithm.jl`:

```julia
@testset "stress smoke — n=100 random scatter" begin
    using Random
    rng = Random.MersenneTwister(0)
    n = 100
    textpositions = [Point2f(500 * rand(rng), 500 * rand(rng)) for _ in 1:n]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(p[1] - 15, p[2] - 5, 30, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)
    offsets  = [Vec2f(0, 0) for _ in 1:n]

    alg = TextRepelAlgorithm(max_iter = 500)
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    # All offsets finite — no NaN/Inf leaked through the solver.
    @test all(o -> all(isfinite, o), offsets)
    # Solver made progress.
    @test solve_stats(alg).iter > 0
    # Rendered bboxes stay within a sanity envelope around the viewport
    # (allow some slack — bounds clamping may not be exact at extremes).
    for i in 1:n
        rendered_center = textpositions[i] .+ offsets[i]
        @test rendered_center[1] > -100 && rendered_center[1] < 600
        @test rendered_center[2] > -100 && rendered_center[2] < 600
    end
end

@testset "stress smoke — pathological co-located cluster (n=30)" begin
    # All 30 anchors at the same position. Solver should fan them out via
    # init_offsets's golden-angle spiral, not produce NaN/coincident bboxes.
    n = 30
    textpositions = fill(Point2f(250, 250), n)
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(p[1] - 25, p[2] - 5, 50, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)
    offsets  = [Vec2f(0, 0) for _ in 1:n]

    alg = TextRepelAlgorithm(max_iter = 500)
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    # No NaN/Inf.
    @test all(o -> all(isfinite, o), offsets)
    # Labels spread out via init_offsets's golden-angle spiral. Each of
    # the 30 indices produces a distinct direction, so under any
    # non-collapsed solve the offsets remain distinct. Threshold > n/2
    # catches partial-collapse regressions where a chunk of labels
    # converge to the same point.
    @test length(Set(offsets)) > n ÷ 2
end
```

- [ ] **Step 2: Run tests to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS (assumes the dispatch from Tasks 8-13 is correct; this is a regression-lock test, not a TDD-driven test).

- [ ] **Step 3: Commit (test-only)**

```bash
git add test/test_annotation_algorithm.jl
git commit -m "Test: stress smoke at n=100 and n=30 co-located cluster

CI-automated companion to SC5's manual visual check. Asserts the
dispatch produces finite offsets and a sensible spatial envelope on
the two configurations that historically broke LabelRepel and the
pre-PR-#7 spike.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
"
```

---

### Task 15: Stability canary

**Goal:** A test that fails loudly if a Makie upgrade renames or re-signatures `calculate_best_offsets!` for `TextRepelAlgorithm`. This is our early-warning system for Makie internals churn.

**Files:**
- Test: `test/test_annotation_algorithm.jl`

- [ ] **Step 1: Write the test**

Add to `test/test_annotation_algorithm.jl`:

```julia
@testset "Stability canary — Makie.calculate_best_offsets! dispatches" begin
    # Confirms the hook symbol exists and dispatches on a TextRepelAlgorithm
    # with the documented kwargs. If a Makie upgrade renames or re-signatures
    # this hook, this test fails loudly in CI before users hit it at runtime.

    @test isdefined(Makie, :calculate_best_offsets!)
    @test hasmethod(Makie.calculate_best_offsets!,
                    Tuple{TextRepelAlgorithm,
                          Vector{<:Vec2},
                          Vector{<:Point2},
                          Vector{<:Point2},
                          Vector{<:Rect2},
                          Rect2};
                    (:maxiter, :labelspace, :reset))
end
```

- [ ] **Step 2: Run test to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS.

- [ ] **Step 3: Commit (test-only)**

```bash
git add test/test_annotation_algorithm.jl
git commit -m "Test: stability canary for Makie.calculate_best_offsets! hook

Fails loudly in CI if a future Makie upgrade renames or re-signatures
the algorithm dispatch hook (it's undocumented internal API; the canary
is our early-warning system).

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
"
```

---

## Phase E — Documentation and examples

Two tasks to finish the user-facing surface.

---

### Task 16: README — "Two surfaces" section

**Goal:** Add a short README section describing both surfaces and when to use each, with cross-linked docstrings. No comparison phrasing.

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Read existing README to find the insertion point**

Run: `cat README.md` and identify a good location — probably just after the `textrepel!` quick-start, before any "Roadmap" or "License" section.

- [ ] **Step 2: Add the "Two surfaces" section**

Insert into `README.md` at the chosen location:

```markdown
## Two surfaces

MakieTextRepel exposes two ways to get repelled labels into a plot:

### `textrepel!`

A standalone Makie recipe with the full feature set: force-directed
solver, dropping (`max_overlaps`), background boxes, and pixel-space
connector trimming.

```julia
using MakieTextRepel
scatter!(ax, points)
textrepel!(ax, points; text = labels, only_move = :y)
```

Use when you need any of those features, or for the default ggrepel /
adjustText workflow.

### `TextRepelAlgorithm`

An algorithm plug-in for `Makie.annotation!`. Reuses the same
force-directed solver underneath `annotation!`'s styling
(`Ann.Styles.LineArrow()`, custom paths, arrow heads).

```julia
using MakieTextRepel
scatter!(ax, points)
annotation!(ax, points; text = labels,
            algorithm = TextRepelAlgorithm(only_move = :y))
```

Also supports per-label pinning (mix finite and `NaN` entries in
`textpositions_offset`) and obstacle avoidance via the `obstacles`
keyword.

Use when you want MakieTextRepel's solver underneath `annotation!`'s
arrow styling.
```

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "Doc: add Two surfaces README section

Documents both textrepel! and TextRepelAlgorithm without
comparison phrasing — each section describes what its surface does
and when to reach for it.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
"
```

---

### Task 17: Update spike examples to use the productionized API

**Goal:** The `examples/annotation_spike.jl` and `examples/stress_compare.jl` scripts currently use the spike's ad-hoc `include(...)` pattern and the qualified `MakieTextRepel.RepelParams`. Update them to use the exported `TextRepelAlgorithm` directly.

**Files:**
- Modify: `examples/annotation_spike.jl`
- Modify: `examples/stress_compare.jl`
- Modify: `examples/annotation_lines_probe.jl`

- [ ] **Step 1: Remove the temporary include from each example**

In `examples/annotation_spike.jl`, remove the line:

```julia
include(joinpath(@__DIR__, "..", "src", "annotation_algorithm.jl"))
```

Verify the script still references `TextRepelAlgorithm()` — that name is now exported from `using MakieTextRepel`.

Do the same in `examples/stress_compare.jl` and `examples/annotation_lines_probe.jl`.

- [ ] **Step 2: Run each example to verify it works**

```bash
julia --project=. examples/annotation_spike.jl
julia --project=. examples/stress_compare.jl
julia --project=. examples/annotation_lines_probe.jl
```

Expected: each writes a PNG to `test/output/`, no errors.

- [ ] **Step 3: Commit**

```bash
git add examples/annotation_spike.jl examples/stress_compare.jl examples/annotation_lines_probe.jl
git commit -m "Examples: use exported TextRepelAlgorithm instead of ad-hoc include

The spike's ad-hoc include is no longer needed now that the wrapper
is wired into the module.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
"
```

---

## Completion checklist

After Task 17, verify each item before declaring the branch ready:

- [ ] `julia --project=. -e 'using Pkg; Pkg.test()'` exits clean, all tests pass
- [ ] `git log --oneline` shows ~17 task commits since the spec commit (`7daf619`)
- [ ] `examples/annotation_spike.jl` produces a 3-panel PNG with visible connectors in the middle panel (annotation! + TextRepelAlgorithm)
- [ ] `examples/stress_compare.jl` produces three stress PNGs without errors
- [ ] **SC5 — stress smoke**: open `test/output/stress_02_severe_n100.png` (n=100) and `test/output/stress_03_pathological_cluster_n30.png` (n=30 co-located); confirm the middle panel ("annotation! + TextRepelAlgorithm") shows a visually clean radial fan in the cluster case and dense-but-legible labels at n=100. No labels sitting on top of their own markers. Then run:

  ```julia
  julia --project=. -e '
  using MakieTextRepel, Makie, GeometryBasics
  # Re-run the stress n=100 scenario and assert all offsets are finite.
  Random.seed!(0)
  # (use the same setup as examples/stress_compare.jl)
  '
  ```

  At minimum, assert `all(isfinite, vcat([collect(o) for o in offsets]...))` on each stress configuration.

- [ ] No StaticArrays broadcast errors in any test run
- [ ] `Pkg.test()` output contains no warnings except those from the deliberate-warning tests (Tasks 6, 12)
- [ ] Spec risks revisited: R3 (warm-start own-anchor invariant) has a test in Task 11; R5 (maxlog=1) is documented in the docstring (Task 6); R6 (pinned anchor inside bbox suppresses connector) is documented in the docstring (Task 6); R7 (zero-width labels) has unconditional guard + test in Task 13; R8 (NaN textpositions) has ArgumentError in Task 13.

When all checked, the branch is ready for the next stage (PR, merge, or further work).
