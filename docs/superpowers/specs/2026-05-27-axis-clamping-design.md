# Axis Clamping for MakieTextRepel — Design

**Date:** 2026-05-27
**Status:** Approved design, pre-implementation
**Follows:** `2026-05-27-makie-text-repel-design.md` (the base package)

## Goal

Keep repelled labels inside the axis plotting area so they don't clip past the
edges. Labels are rendered in pixel space at data anchors; near the data extremes
they currently spill outside the axis and get cut off (e.g. the README demo needed
manual `xlims!/ylims!` padding to look right).

## Background (validated by research)

- **Makie has no native mechanism** to reserve fixed *pixel* space for in-plot
  content. `xautolimitmargin`/`yautolimitmargin` are *fractional* (a percentage of
  the data range), so they can't express a fixed label size and break on figure
  resize. Protrusions are for axis decorations only. There is no hook to expand
  limits "by N pixels" and no fixed-point limit iteration.
- **Makie's own `textlabel` recipe clips identically** — it defines anchors-only
  `data_limits`/`boundingbox` (exactly as MakieTextRepel does) and its pixel-space
  text/box spill past the axis too. Edge-clipping for pixel-space text is an accepted
  Makie limitation; pixel-aware label-fitting is a real gap MakieTextRepel can fill.
- **ggrepel and adjustText both solve this by clamping**, not expanding: each solver
  iteration, a label box poking outside the plotting area is slid back so its edge
  sits on the boundary (box size preserved) — ggrepel's `put_within_bounds`,
  adjustText's `force_into_bbox`. ggrepel *only* clamps (it removed its old soft
  boundary force); adjustText clamps by default with axis-expansion opt-in
  (`expand_axes=False`). ggrepel users expect clamp-to-panel, not a zooming view.

## Decision

**Clamp labels to the axis viewport, on by default.** This matches the references and
user expectations, fits our existing pixel-space solver, keeps data limits stable
(no circular limit-feedback), and is reactive on resize for free. No opt-out
attribute in v1 (no consumers yet; YAGNI) — an overflow escape hatch can be added
later. Axis expansion (the adjustText `expand_axes` philosophy) is explicitly **not**
built.

## Architecture

The clamp is **pure geometry inside the solver**, fed the viewport as data. TextMeasure
already supplies the accurate per-label pixel box sizes the clamp relies on; this
feature needs nothing new from it.

### Solver (`src/solver.jl`)

- Add an optional field to `RepelParams`:
  ```julia
  bounds::Union{Rect2f, Nothing} = nothing   # clamp region in solver (pixel) space; nothing = no clamp
  ```
  `nothing` preserves today's behavior and keeps `solve_repel` testable in isolation
  without a viewport.
- In the `solve_repel` iteration loop, **after** applying each iteration's offset and
  before the convergence check, if `bounds !== nothing` clamp every label's **padded
  box** (`psize = size + 2·box_padding`, the same box used for repulsion) to sit fully
  inside `bounds` (slide inward, preserve size), writing the result back into
  `offsets[i]`. Clamping the *padded* box means the visible label settles `box_padding`
  inside the edge — the gutter emerges from the existing padding, with no separate inset
  (avoids double-counting). Clamping *inside* the loop (not as a one-shot post-pass) lets
  repulsion and confinement reach equilibrium together so edges don't re-pile.
- **Step-cap cooling (convergence fix).** Decay the per-iteration step cap linearly over
  the run: at iteration `it`, use `step_max · max(0, 1 − it/max_iter)` instead of a
  constant `step_max`. Without this, tightly edge-crowded labels pinned against a
  boundary settle into a period-2 limit cycle (overlap-push vs. spring) and spin to
  `max_iter` rather than converging. Cooling is applied **only on the clamped path**
  (`bounds !== nothing`): the recipe always sets `bounds`, so every real `textrepel!`
  call gets the fix, while the bare `bounds = nothing` solver path keeps a constant
  step cap and stays **byte-identical** to its pre-clamping output (no consumer relies
  on the unclamped path, and gating cooling there avoids silently changing shipped
  behavior). Cooling is deterministic (no new parameter) and has no effect on
  non-crowded clamped cases, which converge in a few iterations before it bites.

### Clamp helper (`src/geometry.jl`)

A pure helper, e.g.:
```julia
# Shift `box` minimally so it lies inside `bounds` (preserving size); if `box` is
# larger than `bounds` on an axis, pin it to the lower edge on that axis.
clamp_box_offset(box::Rect2f, bounds::Rect2f) -> Vec2f   # the corrective shift
```
The solver applies the shift to the label's offset. Lives in geometry.jl with the
other pure AABB helpers; no Makie dependency.

### Recipe (`src/recipe.jl`)

- Obtain the plot's **scene viewport** as an observable via
  `Makie.viewport(Makie.parent_scene(p))` (an `Observable{Rect2i}`). `parent_scene(p)`
  is the axis's *data-area* scene — it already excludes ticks/labels/title, so no
  decoration subtraction is needed.
- Build the clamp rectangle from the viewport's **size only**, discarding its origin:
  ```julia
  bounds_obs = lift(Makie.viewport(Makie.parent_scene(p))) do vp
      Rect2f(0, 0, Float32.(widths(vp))...)
  end
  ```
  **Critical:** the viewport `Rect2i` carries a *figure-relative* origin (the axis's
  offset within the figure), but the `:pixel`-projected anchors are *scene-local*
  (origin at the axis lower-left, always in `(0,0)–(w,h)`). Clamping against the raw
  viewport rect would be off by the origin; we must use `(0, 0)–widths(vp)`. (Validated:
  Makie's `pixel_space` is built from `widths(viewport)` only and never uses the origin.)
- Add `bounds_obs` as a **new dependency of the solve `lift`** and pass its value as
  `RepelParams.bounds`. The viewport observable updates on figure resize/layout, so the
  lift re-runs → labels re-solve and re-clamp — the first reactive behavior, for free.

## Testing

- **Pure solver** (core): with an explicit `bounds`, assert every final box lies fully
  within it; box sizes are unchanged; a label seeded near/over an edge ends up pulled
  inside. Include a degenerate case (label larger than bounds → pinned to lower edge,
  no NaN; anchor outside bounds → still lands inside). `bounds = nothing` must be
  byte-identical to today's output. Existing determinism / no-overlap tests still pass.
- **Convergence (cooling)**: a tightly edge-crowded case converges (`maxmove < tol`)
  within `max_iter` *and* stays in-bounds — guards the step-cap-cooling fix. (Without
  cooling this case spins to `max_iter` in a limit cycle.)
- **Clamp helper**: unit tests for box inside (zero shift), box over each edge (correct
  inward shift), box larger than bounds (pinned to lower edge).
- **Integration (CairoMakie)**: with a known figure/viewport, assert the recipe's
  `computed_offsets` keep all label boxes inside the scene viewport.
- **Demo**: regenerate the README image and confirm labels no longer clip *without* the
  manual `xlims!/ylims!` padding (a modest autolimit margin may still aid breathing
  room, but clipping must be gone). Visual confirmation of the feature.

## Out of scope

- **Axis expansion** (adjustText `expand_axes`) — explicitly not built; clamp only.
- **Overflow opt-out attribute** — deferred until a consumer needs it.
- **Own-point repulsion** (labels not covering their own markers) — tracked separately
  in [jowch/MakieTextRepel.jl#1](https://github.com/jowch/MakieTextRepel.jl/issues/1);
  has its own determinism-vs-nudge-direction design.
