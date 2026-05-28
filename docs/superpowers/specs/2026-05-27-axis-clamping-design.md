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

- Obtain the plot's **scene viewport** as an observable. Anchors are already in
  scene-local pixels (origin at the axis lower-left, via `register_projected_positions!`),
  so the clamp rectangle in that frame is the raw viewport `Rect2f(0, 0, vp_width, vp_height)`
  — no inset (the solver clamps the *padded* box, which produces the edge gutter).
- Add the viewport observable as a **new dependency of the solve `lift`** and pass the
  viewport rectangle as `RepelParams.bounds`. The lift re-runs on viewport change, so
  labels re-solve and re-clamp on figure resize/layout — the first reactive behavior,
  obtained for free.

## Testing

- **Pure solver** (core): with an explicit `bounds`, assert every final box lies fully
  within it; box sizes are unchanged; a label seeded near/over an edge ends up pulled
  inside. Include a degenerate case (label larger than bounds → pinned to lower edge,
  no NaN). Existing determinism / no-overlap tests still pass.
- **Clamp helper**: unit tests for box inside (zero shift), box over each edge (correct
  inward shift), box larger than bounds (pinned).
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
