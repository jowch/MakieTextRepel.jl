# Design: Share the placement strategy between `textrepel!` and `TextRepelAlgorithm`

**Issue:** #12
**Date:** 2026-05-28
**Status:** Approved (pending spec review) → implementation plan to follow

## Problem

`textrepel!` (the recipe) and `TextRepelAlgorithm` (the `annotation!` plug-in) share the
same force core (`solve_repel`) but produce **different label placements** for the same
data. The divergence is not in the solver — it is that the v0.2 placement *strategy*
(Voronoi-informed init + leader-line crossing-repair) was wired **inline into the recipe**
and never reached the annotation path.

- **Recipe** (`src/recipe.jl:89-92`):
  `voronoi_cells` → `initial_offsets` → `solve_cluster(ForceSolver, …)` → `repair_crossings!`
- **Annotation** (`src/annotation_algorithm.jl:174-187`):
  `init_offsets` (golden-angle spiral) → `solve_repel` directly, **no repair**.

So the annotation surface effectively still runs pre-v0.2 placement.

### Root cause (the real smell)

The `AbstractClusterSolver` seam (`src/solvers/abstract.jl`) was *designed* to be the home
of a cluster-placement strategy. But the v0.2 strategy got **split across the seam**:
`voronoi_cells` + `initial_offsets` + `repair_crossings!` were left sitting *above* the
solver in the recipe, while `ForceSolver.solve_cluster` owned only the force iteration. The
annotation path then re-implemented a *different, incomplete* strategy inline. A free-floating
`repel_pipeline()` helper (the literal sketch in #12) would add a **third** home for strategy
logic — patching around the seam instead of using it.

## Goal

Make the **full placement strategy a property of the solver seam.** Both surfaces call one
`solve_cluster(...)` and get identical strategy behavior — Voronoi-init, force, repair,
pinning, dropping. Any future change to the strategy then flows to both surfaces through a
single edit site.

## Non-goals

- Changing the force model, Voronoi math, Imhof slot selection, or crossing-repair *algorithms*.
  This is a refactor of *where strategy is orchestrated*, not *what it computes*.
- Unifying the irreducible per-surface adapters (coordinate space, size source, rendering) —
  see "What stays per-surface".
- Hysteresis-gated repair during interaction (a possible future refinement; out of scope).

## Key behavioral decisions

1. **Fresh vs. relax axis, encoded as `init_state`.**
   - `init_state === nothing` ⇒ **fresh** placement: the solver computes Voronoi-init **and**
     runs crossing-repair.
   - `init_state` given ⇒ **relax**: warm-start from those offsets, solve only — **no** Voronoi
     re-init (would clobber the warm state and cause jitter) and **no** repair (swap-based
     repair is discontinuous and can *flicker* frame-to-frame during interaction).

   This maps cleanly onto both surfaces: the recipe always does fresh placement (passes
   `nothing`); the annotation path passes `nothing` on `reset=true` and the warm offsets on
   `reset=false`.

2. **Pinned labels are an absolute guarantee.**
   - `initial_offsets` seeds pinned indices at their pinned offset (not an Imhof slot).
   - `repair_crossings!` skips any crossing pair where *either* member is pinned. A crossing
     the user creates by pinning is the user's responsibility; pinned offsets are never moved.

3. **The strategy is fully described by `ForceSolver(params)` + data.** All tunables live in
   `RepelParams`; `solve_cluster` takes no per-call tuning knobs. This is what makes
   "configure the solver once, both surfaces benefit" literally true — a future tunable is a
   new `RepelParams` field with a shared default, nothing more. Consequently
   **`min_segment_length` moves into `RepelParams`** (it already does double duty in the recipe:
   crossing-repair threshold *and* connector suppression, so it is genuinely a
   placement-geometry knob).

## Architecture

### 1. Evolved seam contract (`src/solvers/abstract.jl`, `src/solvers/force.jl`)

```julia
solve_cluster(s::AbstractClusterSolver, anchors, sizes, bounds;
              init_state     = nothing,        # nothing ⇒ FRESH (voronoi-init + repair); given ⇒ RELAX (solve only)
              pin_mask       = nothing,
              pinned_offsets = Vec2f[],
              obstacles      = Rect2f[])
        -> (; offsets, dropped, iter, residual)
```

Changes vs. today:
- `initial_offsets` is **no longer a positional input** — the solver owns init.
- Return widens from `(offsets, dropped)` to a named tuple adding `iter`/`residual`
  (the annotation path needs these for `alg.last_iter` / `alg.last_residual` diagnostics).
- New kwargs forward pinning and obstacles.
- The crossing-repair threshold comes from `params.min_segment_length` (see decision 3), not a kwarg.

`ForceSolver.solve_cluster` body — this **is** the issue's "repel_pipeline", living in the seam:

```julia
function solve_cluster(s::ForceSolver, anchors, sizes, bounds;
                       init_state=nothing, pin_mask=nothing,
                       pinned_offsets=Vec2f[], obstacles=Rect2f[])
    p = RepelParams(s.params; bounds = bounds)
    fresh = init_state === nothing
    init = fresh ?
        initial_offsets(anchors, sizes, voronoi_cells(anchors, bounds), p;
                        pin_mask, pinned_offsets) :
        init_state
    r = solve_repel(anchors, sizes, p;
                    init_state = init, obstacles, pin_mask, pinned_offsets)
    fresh && repair_crossings!(r.offsets, anchors, sizes, r.dropped, p;
                               min_len = p.min_segment_length, pin_mask)
    return (; r.offsets, r.dropped, r.iter, r.residual)
end
```

Relax mode skips `voronoi_cells` (a Delaunay triangulation — a per-frame perf win during
interaction) and repair.

### 2. Pin-awareness in the two pure steps

- **`initial_offsets`** (`src/init.jl`) gains optional `pin_mask`/`pinned_offsets` kwargs.
  When `pin_mask === nothing` (the recipe's call), behavior is **byte-identical** to today.
  When provided, pinned indices are seeded at `pinned_offsets[i]` instead of an Imhof slot.
- **`repair_crossings!`** (`src/crossings.jl`) gains an optional `pin_mask` kwarg. When
  `nothing`, no pairs are skipped → byte-identical. When provided, a crossing pair is skipped
  if either member is pinned.

### 3. `RepelParams` gains `min_segment_length`

Add a `min_segment_length` field to `RepelParams` (`src/solver.jl`) with a sensible default.
The copy-with-overrides constructor `RepelParams(base; …)` carries it through. This is the
single non-trivial widening of an existing pure type; everything else is additive kwargs.

### 4. Call sites collapse to the seam

**`src/recipe.jl:89-92`** → one call (fresh, no pins, no obstacles):

```julia
offsets, dropped, _, _ = solve_cluster(ForceSolver(params), anchors, sizes, bnds)
```

with `params.min_segment_length` set from the recipe's `min_segment_length` attribute when the
`RepelParams` is built (recipe.jl:81-86). This output **must be byte-identical** to today.

**`src/annotation_algorithm.jl`** → deletes the `if reset … psizes/spiral/align_bias+spiral`
block (lines 174-181) and the direct `solve_repel` call. Keeps: the all-pinned bypass, the
`align_bias` add-in / subtract-out translation, and `pinned_solver`. Becomes roughly:

```julia
init_state = reset ? nothing : Vec2f[offsets[i] + align_bias[i] for i in 1:n]
r = solve_cluster(ForceSolver(effective_params), anchors, sizes, annotation_bounds;
                  init_state, pin_mask, pinned_offsets = pinned_solver,
                  obstacles = alg.obstacles)
alg.last_iter[]     = r.iter
alg.last_residual[] = r.residual
# writeback unchanged: offsets[i] = r.offsets[i] - align_bias[i]
```

The annotation path inherits Voronoi-init + crossing-repair on `reset=true` for free.

## Coordinate-space boundary

The strategy lives entirely in **solver-space** (label-center offsets relative to anchor). The
annotation caller is the only place that translates: it **adds** `align_bias` to every
solver input (warm `init_state`, `pinned_solver`) and **subtracts** it from every writeback.
On the fresh path the caller no longer adds `align_bias` to the init (it passes `nothing`),
because the solver computes the Voronoi init in solver-space itself; the writeback still
subtracts `align_bias`, which is correct. The recipe needs no translation (pixel-space,
centered text).

## What stays per-surface (irreducible adapters)

None of these is strategy logic, and none sits between the caller and the strategy where logic
could leak — the recipe call is a one-liner; the annotation call is a one-liner bracketed by
translation.

| Concern | recipe | annotation |
|---|---|---|
| Coordinate space | pixel, centered (no bias) | render-space + `align_bias` |
| Size source | `measure_labels` | Makie's `text_bbs` |
| Rendering | filters `dropped`; draws text/poly/connectors | writes offsets back to Makie |
| all-pinned shortcut | n/a | early bypass before the solve |

## Structural defenses against re-fragmentation

1. A one-line contract in `src/solvers/abstract.jl`: *"`solve_cluster` owns the entire
   placement strategy (init → force → repair, pin-aware). Callers must not perform
   init/placement/repair outside it."*
2. A test asserting the recipe's computed offsets equal a direct `solve_cluster` call for the
   same inputs — so any future inline strategy leak fails CI.

## Determinism contract

- The recipe path output (offsets + dropped) must be **byte-identical** to pre-refactor for
  the same inputs. Guarded by the existing recipe determinism test and the axis-limit
  regression test, plus the new recipe-equals-`solve_cluster` test.
- The `bounds === nothing` byte-identity contract inside `solve_repel` is untouched (the
  refactor never calls `solve_repel` with `bounds === nothing` on a path that previously had
  bounds).

## Test plan

1. **Recipe byte-identity** — recipe offsets/dropped unchanged vs. a captured baseline;
   recipe output equals a direct `solve_cluster(ForceSolver(params), …)` call.
2. **Pinning × repair** — a pinned label that a swap *would* relocate stays exactly at its
   pinned offset through a fresh solve that triggers repair on the surrounding free labels.
3. **Relax path** — `reset=false` runs solve-only: no Voronoi cells computed, no repair; a
   already-settled, crossing-free warm input is preserved (no swap flicker).
4. **Annotation fresh path is crossing-free** — the #12 acceptance test: `reset=true` over a
   layout that previously produced crossing leaders now produces none.
5. **Coincident anchors, fresh annotation path** — the lost golden-angle-spiral tiebreak risk:
   coincident anchors still separate (relying on the solver's `overlap_push`), labels do not
   collapse onto one another.
6. **Seam contract canary** — update `test_solver.jl`'s `AbstractClusterSolver` /
   `ForceSolver` contract test for the new signature and return shape.
7. **`RepelParams.min_segment_length`** — default present; recipe wires its attribute into the
   field; copy-with-overrides carries it.
8. **Annotation dispatch stability canary** (`test_annotation_algorithm.jl`) — still passes.

## Risks

- **Lost spiral tiebreak** (mitigated by test 5). If coincident anchors fail to separate on the
  fresh annotation path, the follow-up is to add a deterministic golden-angle perturbation to
  the Imhof fallback inside `initial_offsets` — but that would also change recipe output, so it
  must be done carefully behind the same byte-identity guard.
- **`RepelParams` field addition** touches every `RepelParams` constructor call site; the
  copy-with-overrides constructor and any positional construction must be audited.
- **Return-shape change** of `solve_cluster` ripples to its one current caller (recipe) and its
  contract test; both are updated here.
