# Spec: Reuse text measurements across recipe updates

**Issue:** [#25](https://github.com/jowch/MakieTextRepel.jl/issues/25)
**Branch:** `worktree-measure-reuse`
**Date:** 2026-06-18

## Problem

`textrepel!` re-measures every label on **every** reactive update. The recipe's
`Makie.plot!` fuses measurement and solving into a single `lift` over 15 inputs
(`src/recipe.jl:76-111`):

```julia
solved = lift(p.px_anchors, p.text, p.fontsize, p.font, p.force, …, p.markersize) do …
    sizes = measure_labels(labels, font, fs, 1.0)   # re-runs on ANY input change
    …
    solve_cluster(ProjectionSolver(params), anchors, sizes, bnds)
end
```

So mutating only `positions[]` (the movie/animation case — TextMeasure.jl's atlas
example dives the camera, moving anchors every frame while label text is constant)
calls `measure_labels` again every frame. That is wasted font-engine work:
TextMeasure.jl's design separates an expensive measure step (`prepare`/`measure_bounds`)
from cheap layout, precisely so callers can "measure once, layout many."

## Root-cause analysis (what depends on what)

A measurement is a pure function of exactly **three** recipe inputs:

| Input | Measure-invalidating? | Why |
|-------|----------------------|-----|
| `text` | **YES** | the string/RichText content |
| `fontsize` | **YES** | glyph metrics scale with size |
| `font` | **YES** | different font ⇒ different metrics |
| `markersize` | no | feeds `eff_pp` (clearance), a *solve* param — never reaches `measure_labels` |
| `px_anchors` | no | positions; solve-only |
| `box_padding`, `point_padding` | no | solve geometry |
| `bounds` (viewport) | no | solve clipping |
| `force`, `force_point`, `force_pull`, `max_iter`, `max_overlaps` | no | inert under `ProjectionSolver`; ForceSolver-only |
| `only_move` | no | solve axis-lock |
| `min_segment_length` | no | connector rendering |

`measure_labels(labels, font, fontsize, ppu) → Vector{Vec2f}` is already render-free
(`src/measure.jl:10`), and `ppu` is hardcoded `1.0` on the recipe path. There is **no
caching today** — `TextMeasure.prepare()` is called and its reusable `Prepared` object
discarded each time.

The `Makie.annotation!` path (`TextRepelAlgorithm`) is unaffected: it receives
pre-measured bboxes from Makie, so it never calls `measure_labels`. This work is
**recipe-only**.

## Approach (Path A: split the reactive node)

Break the single `lift` into two compute nodes that encode the *true* data
dependencies:

```julia
# 2a. Measure — depends ONLY on the three measure-invalidating inputs.
measured_sizes = lift(p.text, p.fontsize, p.font) do labels, fs, font
    measure_labels(labels, font, fs, 1.0)
end

# 2b. Solve — consumes measured_sizes; depends on anchors + solve-only params.
solved = lift(p.px_anchors, measured_sizes, p.force, p.force_point, p.force_pull,
              p.max_iter, p.only_move, p.box_padding, p.point_padding,
              p.max_overlaps, bounds_obs, p.min_segment_length, p.markersize) do px, sizes, …
    anchors = [Point2f(q[1], q[2]) for q in px]
    eff_pp  = …            # unchanged
    params  = RepelParams(; …)
    offsets, dropped, _, _ = solve_cluster(ProjectionSolver(params), anchors, sizes, bnds)
    (; anchors, sizes, offsets, dropped, params)
end
```

Because `measured_sizes` no longer lists `px_anchors`/params as inputs, mutating only
`positions[]` re-fires `solved` (a fresh solve, using the **cached** `sizes`) but never
re-fires `measured_sizes`. This is correctness-by-construction: the framework reuses the
measurement exactly when it is provably reusable. The `solved` named tuple keeps `sizes`
(passed through from the input), so every downstream node (`box_rects`, `seg_points`,
`computed_sizes`, etc.) is unchanged.

### Why Path A and not "inject pre-measured sizes" (Path B)

Path A makes the graph encode true dependencies; it is invisible to users, fully
backward-compatible, adds no public surface, and cannot produce a stale layout. Path B
(a user-supplied `sizes` attribute) is only needed for reuse *across distinct plot
objects*, which a single reactive graph cannot cover, and it introduces a staleness
hazard (caller must keep `sizes` in sync with `text`). Path B is deferred until a
cross-call need is demonstrated (tracked separately if it arises).

## Requirements

1. Mutating only `positions[]` (or limits/viewport) does **not** call `measure_labels`.
2. Mutating `text`, `fontsize`, or `font` **does** re-measure.
3. **Byte-identity:** for any single static plot, computed offsets/dropped are identical
   to the pre-change output. The existing "#12 structural defense" test
   (`test/test_integration.jl:225`) and the coincident-fan-out test (`:241`) must keep
   passing unchanged.
4. The axis-limit-leakage overrides (`data_limits`/`boundingbox`, `src/recipe.jl:164-167`)
   and all `computed_*` exposed nodes keep their current semantics and values.
5. No new dependency. `markersize`-as-clearance behavior unchanged.

## Non-goals (explicitly deferred)

- **Warm-starting the solver** (`solve_cluster(...; init_state=…)`). The seam exists and
  the recipe never uses it, but enabling it would change outputs and break the
  byte-identity requirement. Out of scope; tracked by **issue #24** (its secondary
  "recipe warm-start" ask). This work and #24 are complementary: #25 splits the solve
  node, which makes it the clean place to later thread `init_state`/`obstacles`. The two
  together form "measure once, warm-solve many" in the recipe — but they must not be
  implemented to fight (recipe warm-start is a deliberate per-frame output change; #25
  guarantees sameness only on the fresh-solve path). #24's *primary* ask (export the
  `solve_cluster` primitive) is on a different surface (`src/MakieTextRepel.jl` exports)
  and does not overlap this plan; its Atlas consumer bypasses the recipe entirely.
- **Path B** (user-injected measurements / `Prepared` caching across calls).
- **Exposing `Prepared` objects** from `measure.jl`. We reuse the *result* `Vec2f` sizes,
  which is sufficient for the recipe; the deeper TextMeasure `Prepared` reuse is a
  measure.jl-level optimization not required here.

## Test strategy

Signal the reuse property with **object identity**, requiring no production test-state.
The split threads the measured-sizes `Vector{Vec2f}` unchanged through `solved`, so the
already-exposed `computed_sizes` node returns:
- the **same object** (`===`) after a position-only update (measurement reused), and
- a **new object** (`!==`) after a `text`/`fontsize`/`font` change (re-measured).

This is also a genuine red→green: today's fused lift re-measures on every update,
producing a fresh `Vector` each time, so the `===` assertion fails pre-fix.

Acceptance test shape (full code in the plan):
- build a plot, `update_state_before_display!`, capture `sizes_before = pl.computed_sizes[]`
- mutate `positions[]`, update → assert `pl.computed_sizes[] === sizes_before`
- mutate `text[]`, update → assert `pl.computed_sizes[] !== sizes_before`
- byte-identity vs a direct `solve_cluster` is already covered by the existing #12 test;
  we keep it green.

## Affected files

- `src/recipe.jl` — split the `lift` (the only behavioral change; no other `src/` file changes).
- `test/test_integration.jl` — add the reuse test; keep #12 + coincident tests green.
- `docs/` + `examples/` — add a movie/animation example demonstrating the pattern and a
  short note in `docs/algorithm.md` or README (the reuse property).
