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

Changes vs. today (the `src/solvers/abstract.jl` docstring — which currently documents
`solve_cluster(s, anchors, sizes, initial_offsets, bounds) → (offsets, dropped)` — must be
updated to this new signature/return as part of the work):
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

Add a `min_segment_length::Float64` field to `RepelParams` (`src/solver.jl`) with default
**`2.0`** — this **must** equal the recipe's `@recipe` `min_segment_length` attribute default
(`src/recipe.jl:31`), or recipe output changes. The `@kwdef`-based copy-with-overrides
constructor `RepelParams(base; …)` carries it through automatically (it enumerates
`fieldnames` dynamically), so all existing construction sites are keyword-only and safe — none
are positional (verified across `src/` and `test/`).

**Scope of "single source of truth":** the field is the strategy's notion of minimum meaningful
leader length, consumed by `repair_crossings!` **inside** `solve_cluster`. It does **not**
change connector *rendering*:

- `build_connectors` / `connector_for` (`src/connectors.jl`) keep their existing positional
  `min_len::Real` argument. The recipe still passes `Float64(ml)` to `build_connectors`
  (`src/recipe.jl:131-134`) from the same attribute — so the repair threshold (via `params`)
  and the connector-suppression threshold (via the explicit arg) read the **same value** and
  stay consistent, but the connector path is not refactored.
- The annotation surface has no connectors of its own (`annotation!` draws them via
  `Ann.Styles`), so for that surface `min_segment_length` affects only crossing-repair. Because
  `TextRepelAlgorithm`'s kwarg constructor validates against `fieldnames(RepelParams)`
  (`src/annotation_algorithm.jl:51-67`), adding the field automatically lets annotation users
  pass `min_segment_length = …`; it will tune repair only. This is acceptable and intended.

### 4. Call sites collapse to the seam

**`src/recipe.jl`.** Only the four-line `init + solve_cluster + repair` block
(`recipe.jl:89-92`) collapses. The surrounding `lift` body is **unchanged**: it still measures
sizes, builds `params`, and **must still return the same local named tuple**
`(; anchors, sizes, offsets, dropped, params)` (`recipe.jl:93`) that five downstream `lift`s
and four `add_input!`s consume (`s.anchors`, `s.sizes`, `s.params`, `s.dropped`). The collapse is:

```julia
# recipe.jl:81-86 — add min_segment_length to the existing kwarg block (no other change):
params = RepelParams(; force = …, …, max_overlaps = Float64(mo), bounds = bnds,
                       min_segment_length = Float64(ml))
# recipe.jl:89-92 — four lines become one; rest of the lift body is untouched:
offsets, dropped, _, _ = solve_cluster(ForceSolver(params), anchors, sizes, bnds)
(; anchors, sizes, offsets, dropped, params)   # unchanged downstream contract
```

Notes:
- `bnds` is passed both inside `params` (line 86, as today) and as the positional `bounds` to
  `solve_cluster`, which re-overrides with the **same value**. This double-set is pre-existing
  (today's `solve_cluster` does `RepelParams(s.params; bounds=bounds)` too) and harmless; kept
  as-is for byte-identity rather than touching the param block.
- `min_segment_length = Float64(ml)` makes the repair threshold inside `solve_cluster` equal
  today's `Float64(ml)` exactly. `build_connectors` (line 131-134) keeps reading `ml` directly,
  so connector suppression is unchanged.
- This output **must be byte-identical** to today (guarded — see Test plan).

**`src/annotation_algorithm.jl`** → deletes the `init_state` computation block **and** the
direct `solve_repel` call (the full span `lines 174-187`: the `if reset … else … end` at
174-181 *and* the `solve_repel(…)` call at 183-187). This also removes the current
`psizes = sizes .+ 2*box_padding` line (175), which is a pre-existing **double-padding bug** —
`init_offsets`/`initial_offsets` pad internally from `params.box_padding`, so feeding
pre-padded sizes padded twice; routing through `solve_cluster` (raw `sizes`) fixes it. Keeps:
the all-pinned bypass, the `align_bias` add-in / subtract-out translation, `pinned_solver`, and
`annotation_bounds` (which retains the `bbox` origin — anchors are in that same frame). Becomes:

```julia
init_state = reset ? nothing : Vec2f[offsets[i] + align_bias[i] for i in 1:n]
r = solve_cluster(ForceSolver(effective_params), anchors, sizes, annotation_bounds;
                  init_state, pin_mask, pinned_offsets = pinned_solver,
                  obstacles = alg.obstacles)
alg.last_iter[]     = r.iter
alg.last_residual[] = r.residual
# writeback unchanged: offsets[i] = r.offsets[i] - align_bias[i]
```

`effective_params` already carries `bounds = annotation_bounds` and `max_iter = mi`;
`solve_cluster` re-overrides `bounds` with the same value (harmless, as above). The annotation
path inherits Voronoi-init + crossing-repair on `reset=true` for free.

## Coordinate-space boundary

The strategy lives entirely in **solver-space** (label-center offsets relative to anchor). The
annotation caller is the only place that translates: it **adds** `align_bias` to every
solver input (warm `init_state`, `pinned_solver`) and **subtracts** it from every writeback.
On the fresh path the caller no longer adds `align_bias` to the init (it passes `nothing`),
because the solver computes the Voronoi init in solver-space itself; the writeback still
subtracts `align_bias`, which is correct. The recipe needs no translation (pixel-space,
centered text).

### Why dropping the fresh-path `align_bias` init-bias is correct for non-centered text

A design review raised a concern that, for non-centered (aligned) text where `align_bias ≠ 0`,
not biasing the fresh init by `align_bias` would make the spring pull labels to the wrong
equilibrium (rendered offset `−align_bias`). Investigated and **dismissed**, on two verified
code facts:

1. **Labels are repelled off their own anchor** (`solver.jl:154-159`): the force loop includes
   each label's own anchor ("keeps isolated labels off their own point"), balanced by the weak,
   thresholded `force_pull` spring. So a label does **not** converge to `offset ≈ 0`; the
   concern's premise (convergence to zero) does not hold.
2. **The writeback is alignment-independent.** With `align_bias = o_bb + w/2 − anchor` and
   `render_offset = solver_offset − align_bias`, the rendered box center is
   `o_bb + render_offset + w/2 = anchor + solver_offset` for **any** alignment. The solver
   reasons purely about box centers; `align_bias` only converts box-center ↔ box-origin at the
   boundary.

Imhof slots are already box-center offsets (`init.jl:7`, `slot_offset`), so the Voronoi init is
in solver-space directly and needs no `align_bias`. The real effect of the change is that the
fresh annotation path adopts Imhof-slot placement (like the recipe) instead of the old
spiral-from-bias — **the intended behavior change of #12**, not a regression. Test plan item 5
guards that non-centered fresh placement is sane.

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

1. **Recipe byte-identity** — recipe offsets/dropped unchanged vs. a captured baseline; and the
   recipe's `lift` output equals a direct `solve_cluster(ForceSolver(params), anchors, sizes, bnds)`
   call (structural defense against future inline leaks).
2. **Pinning × repair (mixed)** — with 2 of N labels pinned on a **fresh** solve, pinned labels
   stay exactly at their pinned offset through a repair that would otherwise swap them; free
   labels still get repaired. Exercises `initial_offsets` pin-seeding + `solve_repel` pin-hold +
   `repair_crossings!` pin-skip composing in the mixed case.
3. **Relax path skips voronoi + repair** — assert the `reset=false` output is **identical** to
   `solve_repel(anchors, sizes, p; init_state = warm_init, obstacles, pin_mask, pinned_offsets)`
   with the **same** auxiliary kwargs as the `solve_cluster` call (not an empty/no-kwarg call —
   that would pass trivially). Equality to this matched direct `solve_repel` is the mechanism
   proving voronoi/repair were bypassed (either would perturb it). Also: an already-settled
   crossing-free warm input is preserved (no swap flicker).
4. **Annotation fresh path is crossing-free** — the #12 acceptance test: `reset=true` over a
   layout that previously produced crossing leaders now produces none.
5. **Coincident anchors + non-centered text, fresh annotation path** — the lost spiral-tiebreak
   risk: coincident anchors still separate (relying on `overlap_push`), labels do not collapse;
   and a non-centered (aligned) label renders sanely (guards the dismissed Risk 3).
6. **Annotation fresh-path determinism** — same inputs over repeated `reset=true` calls give
   byte-identical offsets (voronoi uses a seeded RNG; separate from item 4 — a layout can be
   crossing-free yet non-deterministic).
7. **`obstacles` forwarded through `solve_cluster`** — unit test: `solve_cluster(…; obstacles=[ob])`
   asserts the label avoids `ob`. Existing obstacle tests bypass the seam, so this path is
   currently untested.
8. **All-pinned bypass preserved** — annotation all-pinned early return unchanged:
   `solve_cluster` never called, offsets pass through verbatim, diagnostics zeroed.
9. **`RepelParams.min_segment_length`** — default is exactly `2.0` (matches recipe attr
   `recipe.jl:31`); recipe wires `Float64(ml)` into the field; copy-with-overrides carries it; a
   `TextRepelAlgorithm(; min_segment_length=…)` kwarg is accepted (auto-validated).
10. **Seam contract canary** — update `test_solver.jl`'s `ForceSolver wraps solve_repel` testset
    (`:355`) for the new signature/return: `o1, d1 = …` destructure breaks, arg order changes,
    `init` positional removed.
11. **`repair_crossings!` signature** — `pin_mask` added optional (`=nothing`), `min_len` stays
    required → existing `test_crossings.jl` callers compile unchanged.
12. **Annotation dispatch stability canary** (`test_annotation_algorithm.jl`) — still passes.

## Risks

- **Lost spiral tiebreak** (mitigated by test 5). If coincident anchors fail to separate on the
  fresh annotation path, the follow-up is to add a deterministic golden-angle perturbation to
  the Imhof fallback inside `initial_offsets` — but that would also change recipe output, so it
  must be done carefully behind the same byte-identity guard.
- **`min_segment_length` default mismatch** is the sharpest byte-identity hazard: the new
  `RepelParams` default and the recipe attribute default must both be `2.0`, and the recipe must
  actually wire `Float64(ml)` into the field. Guarded by tests 1 and 9.
- **`solve_cluster` signature change** (drops the `initial_offsets` positional, reorders to
  `bounds`-positional + kwargs, widens to a named-tuple return) ripples to its one src caller
  (recipe) and its test (`test_solver.jl:355`); both updated here. No other src/test/examples
  callers exist (verified).
- **Recipe `lift` body must keep returning `(; anchors, sizes, offsets, dropped, params)`** —
  `solve_cluster` returns only `(; offsets, dropped, iter, residual)`, so the surrounding tuple
  is reassembled in the `lift`, not taken from the call. Downstream `lift`s/`add_input!`s depend
  on it.
- **Pre-existing double-padding bug** in the annotation reset path (`psizes` fed to a
  self-padding init) is removed by this refactor; watch for a placement-spread shift in any
  annotation snapshot baselines.
