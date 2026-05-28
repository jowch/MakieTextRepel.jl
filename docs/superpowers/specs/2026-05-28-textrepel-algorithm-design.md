# `TextRepelAlgorithm`: design spec

Date: 2026-05-28
Status: Draft v2 (post-review)
Branch: `worktree-annotation-algorithm-spike`

## Goal

Ship `TextRepelAlgorithm`: an algorithm plug-in for `Makie.annotation!` that
delegates label placement to MakieTextRepel's force-directed solver. The
plug-in is a second supported surface alongside the existing `textrepel!`
recipe, sharing the same solver, exposing every solver capability the
`annotation!` algorithm contract can carry.

## Project context

- MakieTextRepel.jl is a Julia package providing `textrepel!`, a Makie
  recipe for ggrepel/adjustText-style label-repel layouts.
- Makie 0.24.10's `annotation!` recipe exposes an `algorithm` attribute
  that takes a user-supplied object; `Makie.calculate_best_offsets!` is
  the dispatch hook for placement.
- A working spike at `src/annotation_algorithm.jl` (commit `0039c13`) wraps
  the solver as a `TextRepelAlgorithm`. The spike is functional but
  conservative — it accepts a thin slice of solver knobs, assumes
  center-aligned text, ignores `reset` and per-label-pinning signals, and
  is loaded ad-hoc rather than wired into the package.
- PR #7 (merged `aca2312`) hardened the solver: deterministic
  `init_offsets`, own-anchor repulsion, `point_padding` connector trim,
  visible-length filter. These changes propagate to the wrapper for free
  via the shared solver.
- Deep research on `annotation!`'s implementation lives at
  `docs/annotation-research/` (commit `6f2f771`).

## Decisions

### D1 — Wrap a `RepelParams`

`TextRepelAlgorithm` holds a `RepelParams` field. Every solver knob the
recipe accepts is available to the algorithm by construction; future
additions propagate without code changes in the wrapper.

```julia
struct TextRepelAlgorithm
    params::RepelParams
    obstacles::Vector{Rect2f}
    last_iter::Base.RefValue{Int}
    last_residual::Base.RefValue{Float32}
end
```

`Ref`-based mutable diagnostics let `TextRepelAlgorithm` itself remain a
plain immutable struct. Users read diagnostics via the
`solve_stats(alg)` getter (see D9), not by dereferencing the `Ref`s
directly — the `Ref` typing is an implementation detail.

### D2 — Two constructors

```julia
# Kwarg-forwarding: builds a RepelParams internally
TextRepelAlgorithm(; obstacles = Rect2f[], bounds = nothing, kwargs...)

# Explicit: accepts a fully-formed RepelParams
TextRepelAlgorithm(params::RepelParams; obstacles = Rect2f[])
```

The kwarg form covers the common case. The explicit form is for code
that shares one `RepelParams` between `textrepel!` and the algorithm
plug-in.

The kwarg constructor validates unknown kwargs up front:

```julia
extra = setdiff(keys(kwargs), fieldnames(RepelParams))
isempty(extra) || throw(ArgumentError(
    "TextRepelAlgorithm: unknown keyword(s) $(collect(extra)). \
     Valid: $(fieldnames(RepelParams)), plus `bounds`, `obstacles`."))
```

This catches typos like `force_pul = (.02, .02)` at the wrapper
boundary instead of producing a cryptic `MethodError` from inside
`Base.@kwdef`-generated code.

### D3 — `bounds` is solver-managed, not user-facing

The wrapper unconditionally overrides `params.bounds` with annotation's
`bbox` at solve time. The kwarg constructor accepts `bounds = nothing`
as the default; any other value triggers a `@warn` once per session
(`maxlog = 1`) and is then ignored.

Rationale: viewport is intrinsically a pixel-space concept, but users
think about visible regions in data coordinates. Rather than ship a
half-baked coordinate-conversion API, we hide the knob and let
annotation supply the viewport. A future feature (data-space inset, e.g.
"leave 10% on the right for a legend") can be added when a real use case
arrives.

### D4 — `max_overlaps` warns once and is ignored

Label dropping has no expression in `annotation!`'s algorithm contract
(the `offsets` vector must be fully populated; there is no sentinel for
"skip this label"). The wrapper detects `params.max_overlaps != Inf` at
construction time and emits a single `@warn` per session, then proceeds
with no dropping.

### D5 — Alignment-correct placement via pre-biased init_state

For non-center alignments (e.g., `align = (:left, :center)`),
`text_bbs[i].origin + widths/2 ≠ textpositions[i]`. The label box is
visually centered at `bbox_center[i]`, not at the data point
`textpositions[i]`.

Naively passing `bbox_center` as the solver anchor would cause two
distinct problems: own-anchor repulsion (PR #7's "labels never sit on
their own point" guarantee) would push from `bbox_center` rather than
from the actual data point, and `force_pull` would settle the label
near `bbox_center`. For non-center alignments these can be tens of
pixels apart — visually incorrect.

**Fix: pass `textpositions[i]` as the repulsion anchor, and pre-bias
`init_state[i] = bbox_center[i] - textpositions[i]`** so the box
starts placed correctly at `bbox_center`. The solver's `box_at(anchor,
init_state + Δ, size)` centers the box at `bbox_center + Δ`, and the
returned offset (relative to `textposition`) is what annotation expects.
Repulsion and pull both operate from the actual data point.

This composes cleanly with D7's warm-start: when warm-start is active,
the incoming offsets are already correct deltas relative to
`textpositions`; the alignment pre-bias is only applied when starting
fresh (`reset = true` or first solve).

### D6 — Per-label pinning via mixed-mode `textpositions_offset`

When the algorithm callback receives `textpositions_offset` with a mix
of finite and `NaN` entries, the wrapper treats finite entries as
*pinned*: those labels are placed at the manual offset and act as
immovable obstacles for the solver, which positions the `NaN`-marked
labels around them.

This relaxes the all-or-nothing check that `::Automatic` and
`::LabelRepel` apply. It uses a signal Makie's compute graph already
emits but its built-in algorithms ignore. The compute graph TODO at
`annotation.jl:447-449` confirms this is the intended direction for the
contract.

**Pin × `only_move` semantics**: pinned offsets bypass `only_move`.
The user explicitly supplied a specific `textpositions_offset` value;
constraining it would silently mutate their input. Non-pinned labels
remain `only_move`-constrained as usual.

**Pin × connector-skip gotcha**: if a user pins a label such that the
data anchor lies inside the resulting bbox (which they can do — the
wrapper trusts pinned values), annotation's
`p2 in offset_bb && return` at `annotation.jl:337` will silently
suppress that label's connector. This is a known annotation! behavior,
not a wrapper bug. Document in the docstring.

### D7 — Warm-start on `reset = false`

When the compute graph calls the wrapper with `reset = false`, the
existing `offsets` vector is the previous solution. The wrapper passes
it to `solve_repel` as `init_state` instead of running `init_offsets`,
skipping the alignment pre-bias from D5. The solver runs the same
iteration loop from a near-equilibrium configuration, converging
faster.

**Actual trigger condition**: per `annotation.jl:318`,
`reset = !advance`, and `advance` is true only when
`__advance_optimization` is the sole changed compute-graph input. In
practice this fires when user code explicitly calls
`advance_optimization!(plot)` for iterative refinement. Pan, zoom, and
data updates set `reset = true` and re-run from scratch.

Value claim: smoother convergence for users running interactive
iterative refinement via `advance_optimization!`. Not a pan/zoom
optimization. (If a future Makie minor extends `advance` semantics to
other update paths, this becomes more broadly useful for free.)

### D8 — `obstacles::Vector{Rect2f}` on the algorithm struct

User-supplied axis-aligned rectangles (in pixel space, same coordinate
system as `bbox`) act as additional repulsion sources during solving.
Labels avoid them like they avoid other labels.

Use cases: keep labels off a legend, off a scatter cluster's bounding
region, off an inset axis.

Pixel-space units are documented loudly in the docstring with a
one-liner showing how to derive a pixel-space rect from a data-space
region via `Makie.project`. Data-space input is not exposed in v1;
revisit when a real workflow demands it (mirrors the `bounds` posture
from D3).

This is the one feature where the wrapper exposes a knob the recipe
doesn't (yet); the recipe will inherit it later if a real use case
emerges.

### D9 — Diagnostics via `solve_stats` getter

After each solve, the wrapper updates `alg.last_iter[]` and
`alg.last_residual[]` from `solve_repel`'s return value. Users access
them via a public getter:

```julia
solve_stats(alg::TextRepelAlgorithm) =
    (iter = alg.last_iter[], residual = alg.last_residual[])
```

The `Ref` typing on the struct fields is an implementation detail;
`solve_stats` is the documented interface and what tests assert
against.

### D10 — Deferred for v1

- Per-label `RepelParams` (vector of per-label configs)
- Annealing schedule on the solver
- Custom collision shapes (circles, rotated rects)
- Data-space `obstacles` and `bounds` user knobs

No driving use cases. Add when asked.

## Architecture

One new source file, included after `solver.jl` so `RepelParams` and
`solve_repel` are in scope. One new test file.

```
src/
  MakieTextRepel.jl          # add `include("annotation_algorithm.jl")`
  solver.jl                  # extended: init_state, obstacles, pin_mask,
                             # pinned_offsets kwargs; NamedTuple return
  recipe.jl                  # one-line destructure change for new return
  annotation_algorithm.jl    # NEW: the wrapper
test/
  test_annotation_algorithm.jl   # NEW
  runtests.jl                # includes the new test file
docs/
  annotation-research/       # unchanged; canonical reference for Makie internals
```

`TextRepelAlgorithm` and `solve_stats` are exported at top level
alongside `textrepel!`. No submodule. The existing `textrepel!` recipe
is untouched except for the one-line destructure update (see C1
fix below).

## API

### Struct

```julia
struct TextRepelAlgorithm
    params::RepelParams
    obstacles::Vector{Rect2f}
    last_iter::Base.RefValue{Int}
    last_residual::Base.RefValue{Float32}
end
```

### Constructors

```julia
"""
    TextRepelAlgorithm(; force, force_point, force_pull, only_move,
                         box_padding, point_padding, max_iter,
                         obstacles = Rect2f[], ...)

Kwarg-forwarding constructor. Unknown keywords throw `ArgumentError`.
`bounds` is set automatically from the axis viewport; passing it
triggers a one-time warning. `max_overlaps` has no equivalent under
`annotation!`; passing a non-`Inf` value triggers a one-time warning.
"""
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

"""
    TextRepelAlgorithm(params::RepelParams; obstacles = Rect2f[])

Explicit constructor. Useful when the same `RepelParams` is shared
between `textrepel!` and `annotation!` calls in the same script.
"""
function TextRepelAlgorithm(params::RepelParams;
                            obstacles::Vector{Rect2f} = Rect2f[])
    if params.max_overlaps !== Inf
        @warn "TextRepelAlgorithm: `max_overlaps` has no equivalent under \
            annotation!; use `textrepel!` if you need label dropping." maxlog=1
    end
    return TextRepelAlgorithm(params, obstacles, Ref(0), Ref(0f0))
end
```

### Diagnostics getter

```julia
"""
    solve_stats(alg::TextRepelAlgorithm) -> (; iter, residual)

Return iteration count and final residual from the most recent solve.
Returns `(iter = 0, residual = 0f0)` before any solve runs.
"""
solve_stats(alg::TextRepelAlgorithm) =
    (iter = alg.last_iter[], residual = alg.last_residual[])
```

### Dispatch

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

    # Repulsion anchors are the data points (D5).
    anchors = [Point2f(p[1], p[2]) for p in textpositions]
    sizes   = [Vec2f(bb.widths[1], bb.widths[2]) for bb in text_bbs]
    # bbox_center is where the label box visually sits relative to the anchor.
    bbox_centers = [Point2f(bb.origin[1] + bb.widths[1]/2,
                            bb.origin[2] + bb.widths[2]/2) for bb in text_bbs]
    # Alignment pre-bias (D5): box starts centered at bbox_center.
    align_bias = [Vec2f(c[1] - a[1], c[2] - a[2])
                  for (c, a) in zip(bbox_centers, anchors)]

    annotation_bounds = Rect2f(Float32(bbox.origin[1]),  Float32(bbox.origin[2]),
                               Float32(bbox.widths[1]),  Float32(bbox.widths[2]))

    # Per-label pinning (D6): finite textpositions_offset[i] => pinned.
    pin_mask = BitVector([all(isfinite, p) for p in textpositions_offset])
    pinned_offsets = Vector{Vec2f}(undef, n)
    for i in 1:n
        if pin_mask[i]
            d = textpositions_offset[i] - textpositions[i]
            pinned_offsets[i] = Vec2f(d[1], d[2])  # bypass only_move (D6)
        else
            pinned_offsets[i] = Vec2f(0, 0)
        end
    end

    # All-finite => no solve needed.
    if all(pin_mask)
        for i in 1:n
            offsets[i] = T(pinned_offsets[i][1], pinned_offsets[i][2])
        end
        alg.last_iter[] = 0
        alg.last_residual[] = 0f0
        return
    end

    # Warm-start (D7): use existing offsets when reset == false.
    init_state = if reset
        # Apply alignment pre-bias when starting fresh.
        align_bias
    else
        # Warm-start: incoming offsets are already textposition-relative.
        Vec2f[Vec2f(o[1], o[2]) for o in offsets]
    end

    mi = maxiter === Makie.automatic ? alg.params.max_iter : Int(maxiter)
    effective_params = RepelParams(alg.params;
        bounds   = annotation_bounds,
        max_iter = mi,
    )

    result = solve_repel(anchors, sizes, effective_params;
                         obstacles      = alg.obstacles,
                         pin_mask       = pin_mask,
                         pinned_offsets = pinned_offsets,
                         init_state     = init_state)

    # Diagnostics (D9).
    alg.last_iter[] = result.iter
    alg.last_residual[] = result.residual

    for i in 1:n
        offsets[i] = T(result.offsets[i][1], result.offsets[i][2])
    end
    return
end
```

## Solver changes required

### C1 — Return shape (load-bearing)

Current: `solve_repel` returns `(offsets::Vector{Vec2f},
dropped::BitVector)`. `recipe.jl:87` destructures
`offsets, dropped = solve_repel(...)`.

Change to a NamedTuple:

```julia
return (; offsets, dropped, iter = final_iter, residual = final_residual)
```

Update both call sites:

- `recipe.jl:87` →
  ```julia
  solved = solve_repel(anchors, sizes, params)
  offsets, dropped = solved.offsets, solved.dropped
  ```
  (or equivalently `(; offsets, dropped) = solve_repel(...)`)
- Wrapper (above) accesses `result.offsets`, `result.iter`,
  `result.residual`.

This is backwards-incompatible at the destructure site but
forward-compatible for future diagnostic additions. The recipe gets a
one-line touch; no behavioral change.

### Solver kwargs

`solve_repel` gains four keyword arguments, all with backwards-
compatible defaults:

```julia
function solve_repel(anchors, sizes, params;
                     obstacles::Vector{Rect2f}      = Rect2f[],
                     pin_mask::Union{Nothing,BitVector} = nothing,
                     pinned_offsets::Vector{Vec2f}  = Vec2f[],
                     init_state::Union{Nothing,Vector{Vec2f}} = nothing)
    # ... existing body, modified at four sites:
    #
    # 1. Initial offsets:
    #    offsets = if init_state !== nothing
    #        copy(init_state)
    #    else
    #        [_constrain(o, params.only_move) for o in init_offsets(anchors, psizes, params)]
    #    end
    #
    # 2. Force-application loop: skip force/pull/clamp for pinned indices
    #    (when pin_mask !== nothing && pin_mask[i]). Their offset stays at
    #    pinned_offsets[i] throughout iteration.
    #
    # 3. Pinned labels still contribute their box as a repulsion source
    #    for non-pinned labels (use pinned_offsets[i] to place the box).
    #
    # 4. Obstacle repulsion: extra loop over `obstacles`, applying
    #    point_push from each obstacle rect to each non-pinned label.
    #
    # 5. Return contract: for any i with pin_mask[i] == true, the returned
    #    offsets[i] MUST equal pinned_offsets[i] exactly (no constraint, no
    #    clamping). This is what the wrapper relies on when writing the
    #    final offsets vector with a single `result.offsets[i]` loop.
    return (; offsets, dropped, iter = final_iter, residual = final_residual)
end
```

### Required: `RepelParams` copy-with-overrides constructor

A separate, load-bearing solver-side change. The wrapper needs to
override `bounds` and `max_iter` without rebuilding from scratch:

```julia
RepelParams(base::RepelParams; kwargs...) = RepelParams(;
    (field => get(kwargs, field, getfield(base, field))
     for field in fieldnames(RepelParams))...)
```

The generator-splat is valid with `Base.@kwdef`. Existing positional
recipe call sites are unaffected.

## Documentation

### Docstring on `TextRepelAlgorithm`

```
TextRepelAlgorithm(; force, force_point, force_pull, only_move,
                     box_padding, point_padding, max_iter,
                     obstacles = Rect2f[], ...)
TextRepelAlgorithm(params::RepelParams; obstacles = Rect2f[])

An algorithm plug-in for `Makie.annotation!`. Uses MakieTextRepel's
force-directed solver to place non-overlapping labels around data
points.

Supports per-label pinning: set entries of `textpositions_offset` to
fixed values to lock those labels and let the solver place the rest.
Honors `reset = false` from `annotation!`'s compute graph to warm-start
solves under `advance_optimization!`.

`obstacles` is a `Vector{Rect2f}` in pixel space — the same coordinate
system `annotation!` uses internally. Convert a data-space rectangle
with `Makie.project`:

    px = Makie.project(ax.scene, data_pt)

# Examples

    annotation!(ax, points; text = labels,
                algorithm = TextRepelAlgorithm(only_move = :y))

    # Pin two labels, let the solver place the rest:
    offsets = fill(Point2f(NaN, NaN), length(points))
    offsets[1] = points[1] .+ (50, -20)  # manual placement (pixels)
    offsets[2] = points[2] .+ (-30, 40)
    annotation!(ax, points;
                text = labels,
                textposition_offset = offsets,
                algorithm = TextRepelAlgorithm())

    # Keep labels out of a legend region (pixel-space rect):
    annotation!(ax, points; text = labels,
                algorithm = TextRepelAlgorithm(
                    obstacles = [Rect2f(600, 0, 200, 800)]))

    # Inspect convergence:
    alg = TextRepelAlgorithm()
    annotation!(ax, points; text = labels, algorithm = alg)
    Makie.update_state_before_display!(fig.scene)
    @show solve_stats(alg)  # (iter = 47, residual = 0.0023)

# Scope

`max_overlaps` and background boxes are `textrepel!`-only — they have
no equivalent in `annotation!`'s algorithm contract.

See also: [`textrepel!`](@ref), [`solve_stats`](@ref).
```

### README

A new short section titled "Two surfaces" describing both:

- `textrepel!` — full-featured recipe with dropping, backgrounds, and
  pixel-space connectors. Use when you need any of those features, or
  for the default ggrepel/adjustText workflow.
- `TextRepelAlgorithm` — algorithm plug-in for `annotation!`. Use when
  you want MakieTextRepel's solver underneath `annotation!`'s arrow
  styling (`Ann.Styles.LineArrow()`, custom paths, arrow heads), or
  when you need per-label pinning or obstacle avoidance.

A brief example for each. Cross-link the docstrings.

## Tests

`test/test_annotation_algorithm.jl` covers, at minimum:

### Constructors and warnings

1. **Kwarg form**: defaults; `force`, `only_move`, etc. flow into
   `params`; resulting `TextRepelAlgorithm` is well-formed.
2. **Explicit form**: accepts a `RepelParams`; obstacles default to
   empty.
3. **Unknown kwarg**: `TextRepelAlgorithm(force_pul = (.02, .02))`
   throws `ArgumentError` with a message naming the bad keyword.
4. **`bounds` warning**: `TextRepelAlgorithm(bounds = Rect2f(0,0,1,1))`
   emits one `@warn` (use `@test_logs`).
5. **`max_overlaps` warning**: `TextRepelAlgorithm(max_overlaps = 3)`
   emits one `@warn`.

### Dispatch — edge cases

6. **n=0**: dispatch returns without error, leaves `offsets`
   untouched; `solve_stats` reports `(iter=0, residual=0f0)`.
7. **n=1**: single label placed without crashing.
8. **Coincident anchors**: two labels at the same position fan out via
   `init_offsets`'s golden-angle spiral.
9. **Zero-width label** (`text_bbs[i].widths == (0, 0)`): dispatch
   does not divide by zero; pin or place safely.
10. **NaN in `textpositions`**: dispatch errors or sanitizes; spec
    decision = error with a clear message rather than propagate NaN.

### Dispatch — semantic paths

11. **Manual mode (all-pinned)**: all-finite `textpositions_offset`
    bypasses the solver entirely; offsets exactly match the manual
    deltas; `solve_stats` reports `(iter=0, residual=0f0)`.
12. **Per-label pinning (mixed)**: mixed finite/NaN places NaN labels
    around pinned ones; pinned offsets remain exactly at their input
    values (within float precision); pinned bboxes act as obstacles
    (no NaN-label bbox overlaps a pinned bbox).
13. **Pin × `only_move`**: a pinned offset with non-zero x component
    with `only_move = :y` set — pinned label keeps its x component
    (D6 explicit semantics).
14. **`maxiter` precedence**: explicit `maxiter = 50` passed by
    Makie's compute graph beats `params.max_iter`; `Makie.automatic`
    falls through to `params.max_iter`.
15. **Warm-start**: `reset = false` with a near-equilibrium starting
    state converges in fewer iterations than `reset = true` on the
    same input (assert `solve_stats(alg).iter` drops).
16. **`reset = false` with stale offsets length**: this case shouldn't
    occur in practice (Makie always resizes before calling), but
    document the wrapper assumes the incoming length matches `n`.
    Test with mismatched length asserts a clean error rather than
    garbage layout.
17. **Obstacles — single**: one `Rect2f` obstacle; no returned label
    bbox intersects the obstacle.
18. **Obstacles — multiple**: two disjoint obstacles; same invariant
    for both.
19. **Obstacles — empty vector**: solver path agrees with the
    no-obstacles call (regression check on the kwarg).
20. **Alignment correctness**: `align = (:left, :center)` — returned
    offsets place text bboxes such that own-anchor repulsion is from
    the data point, not from bbox center (asserted indirectly: with a
    single label whose bbox would extend past the anchor if anchored
    incorrectly, the bbox does not cover the anchor).

### Solver-call composition

21. **`labelspace = :data`**: dispatch is invariant under labelspace
    (the solver always works in pixel space; we just pass through).
22. **StaticArrays broadcast canary**: the manual-mode path writes
    `offsets[i] = T(...)` per element. Run with a `Vector{Vec2f}` and
    assert no MethodError or DimensionMismatch from broadcasting.

### Diagnostics

23. **`solve_stats` populated**: after a normal solve,
    `solve_stats(alg).iter > 0` and `residual >= 0`.
24. **`solve_stats` on all-pinned**: returns `(0, 0f0)` per
    explicit early-return in dispatch.

### Stability canary

25. **`Makie.calculate_best_offsets!` exists and dispatches on
    `TextRepelAlgorithm`**: tests that the hook symbol is reachable
    and that calling it with the right arg types doesn't
    MethodError. Fails loudly if a Makie upgrade renames or
    re-signatures the hook.

All tests are CI-friendly (no visual inspection required).

## Compat & guardrails

- `[compat] Makie = "0.24"` in `Project.toml`. Bump deliberately after
  testing on each new Makie minor.
- Stability canary test (#25 above) as runtime guardrail.
- `TextRepelAlgorithm` docstring notes the wrapper uses internal Makie
  API (`Makie.calculate_best_offsets!`).

## Out of scope

- Per-label `RepelParams` (vector configs). Deferred until use case.
- Annealing schedule. Solver-internal improvement, separate effort.
- Custom collision shapes (circles, rotated rects). Solver only
  handles axis-aligned rects.
- Label dropping (`max_overlaps`). Not expressible in `annotation!`'s
  contract; remains a `textrepel!` recipe feature.
- Background boxes. Not expressible in `annotation!`'s contract;
  remains a `textrepel!` recipe feature.
- `bounds` and `obstacles` as data-space or normalized-inset user
  knobs. Pixel-space only in v1; revisit when a real use case arrives.
- Mutating `Project.toml`'s `TextMeasure` source: orthogonal to this
  work; tracked under separate release-blockers memory.

## Success criteria

1. `using MakieTextRepel; TextRepelAlgorithm()` constructs successfully.
2. `annotation!(ax, points; text = labels, algorithm = TextRepelAlgorithm())`
   produces a layout with no own-marker overlap and visible connectors,
   matching the visual quality of `textrepel!()` on the same input.
3. All tests in §"Tests" pass.
4. `Pkg.test()` runs end-to-end with no warnings beyond the deliberate
   `bounds`/`max_overlaps` ones triggered by their dedicated tests.
5. Stress test at n=100 and pathological cluster (n=30 co-located)
   produces a visually clean layout via the wrapper, comparable to
   `textrepel!()` defaults.

## Risks

### R1 — Makie internals churn

`Makie.calculate_best_offsets!` is undocumented; signature or behavior
changes on a minor Makie bump could break the wrapper silently.
Mitigated by the stability canary test (#25) and the explicit Makie
compat pin.

### R2 — Pinning requires nontrivial solver changes

The solver currently has no concept of immovable labels. The spec
calls for adding `pin_mask` and `pinned_offsets` kwargs to
`solve_repel`. If the force-loop changes turn out more invasive than
the four sites enumerated above, pinning becomes a follow-up rather
than v1.

### R3 — Warm-start interaction with `init_offsets`

PR #7 made the default init non-trivial (golden-angle spiral).
Warm-start skips it. We rely on the solver iteration being correct
from any starting state, not just from the spiral init. The "own
anchor" invariant from PR #7 may not hold across a warm-start path
where the incoming offsets are already near-equilibrium. This is
acceptable for `advance_optimization!` users (the only triggering
path) but document the narrowness.

### R4 — `__advance_optimization` semantics depend on Makie internals

D7 relies on `reset = !advance` from `annotation.jl:318`. If Makie
changes the conditions under which `advance` is true (e.g., extends
it to pan/zoom in a future minor), warm-start activates in places we
haven't tested. Stability canary won't catch this — it's a behavioral
change, not a signature change.

### R5 — `maxlog = 1` is process-global

A user constructing two `TextRepelAlgorithm`s in the same session,
both with `max_overlaps = 3`, sees one warning (for the first call)
and silence (for the second). This is the standard Julia logger
contract, but may surprise users. Accept the surprise; document in
the docstring's `max_overlaps` mention.

### R6 — Pinned values that put anchor inside bbox suppress connectors

Annotation's `p2 in offset_bb && return` at `annotation.jl:337`
silently skips the connector when the data point lies inside the
label's offset bbox. With pinning, the user can supply such an
offset. No wrapper recourse — document and move on.

### R7 — Zero-width labels (`""`)

Empty string labels produce zero-width bboxes. Repulsion forces
from zero-width boxes are numerically degenerate. Test #9 covers
this; solver may need a `max(widths, 1f0)` guard at the force-loop
site if degeneracy is observed.

### R8 — NaN/Inf in `textpositions`

Upstream data with NaN/Inf positions propagates into our solver and
produces NaN offsets. Test #10 covers; spec decision: error with a
clear message at the dispatch boundary rather than silently produce
garbage layout.
