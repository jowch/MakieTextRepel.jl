# MakieTextRepel.jl — Design

**Date:** 2026-05-27
**Status:** Approved design, pre-implementation

## Goal

A `ggrepel`/`adjustText`-style label-repel utility for Makie/CairoMakie: given data
points and text labels, automatically displace the labels so they don't overlap each
other or their anchor points, optionally drawing connector lines back to each point.

This is a genuine gap in the Julia plotting ecosystem (no native Makie equivalent
exists). The package builds on [TextMeasure.jl](https://github.com/jowch/TextMeasure.jl)
for accurate, render-free text measurement.

## Key insight: TextMeasure replaces the "spy-shape hack"

Prior research (see `reference-conversation.md`) wrestled with one core problem: how to
measure a label's pixel dimensions in Makie *without rendering it first*. The proposed
workaround was a fragile two-pass "spy-shape" trick (render invisible `textlabel`s with a
`shape` callback to capture bounding boxes, then re-render).

TextMeasure.jl's `MakieBackend` solves this cleanly: `prepare()` + `layout()` return
accurate pixel dimensions from real font metrics, upfront, with no render pass. The
backend is documented to match `Makie.text!()` exactly at `px_per_unit = 1`. This
collapses the architecture into a simple **measure → solve → render once** pipeline and
realizes the "adjustText Option B" approach (pure-Julia solver, measure upfront) — except
the measurement is now *accurate* rather than estimated, which was that approach's main
weakness.

## Verified environment facts (empirically validated against the live stack)

- Installed: **Julia 1.12.6**, **Makie 0.24.10 / CairoMakie 0.15.10**.
- `textlabel` exists; uses the new `@recipe Name (args,) begin … end` style.
- `string_boundingbox` is **deprecated** → use `full_boundingbox` / `text_bb`.
- `register_projected_positions!` is exported — the clean data→pixel path (preferred over
  raw `Makie.project`).
- TextMeasure measures at **word/line** granularity in **pixels**, block-top `y = 0`,
  y increasing downward. No per-glyph boxes, no rotation, no rich-text support.

## Scope decisions

| Decision | Choice | Rationale |
|---|---|---|
| Reactivity | **Static output**, built with clean measure/solve/render seams so a reactive wrapper is a later additive change. | Matches how ggrepel/adjustText work; fits CairoMakie publication workflows; far simpler and testable. |
| Algorithm | **adjustText engine** (pixel space, anisotropic forces, explosion init) + improvements. | Pixel space avoids aspect-ratio distortion; anisotropic control is more useful. |
| Determinism | **Fully deterministic** (no random init/jitter). | Reproducible publication figures — a real improvement over both upstreams. |
| Dropping | **Policy, not default** — `max_overlaps = Inf` keeps all; finite value drops (ggrepel-style). | User chooses legibility-vs-completeness per plot. |
| Connectors | **Edge-clipped** (ggrepel-style geometric intersection with the box edge). | Looks better than center-to-anchor lines. |
| Label style | **Both** plain text (default) and background-boxed, via a `background` flag. Plus **`rich` text** support. | Mirrors ggrepel's `geom_text_repel` / `geom_label_repel` duality. |
| Placement family | **Force-directed for v1**; discrete candidate-position placement noted as future work. | Proven, compact, smooth results. |

### ggrepel vs adjustText (reference)

| Dimension | ggrepel | adjustText | Our choice |
|---|---|---|---|
| Core loop | single physics loop | explosion pre-pass + force loop | explosion + force loop |
| Space | data/native | display/pixel | **pixel** |
| Force control | single scalar | anisotropic x/y | **anisotropic** |
| Overflow | drops > `max.overlaps` | never drops | **policy (default keep)** |
| Determinism | random (seedable) | mostly, random shifts | **fully deterministic** |
| Connectors | edge-clipped | center-ish | **edge-clipped** |

## Architecture

Three isolated stages with clean seams:

```
                 ┌──────────────┐   ┌──────────┐   ┌──────────┐
positions ──────▶│   MEASURE    │──▶│  SOLVE   │──▶│  RENDER  │──▶ plot
labels    ──────▶│ (pixel boxes)│   │ (offsets)│   │text!/box │
                 └──────────────┘   └──────────┘   └──────────┘
                  TextMeasure         pure Julia      Makie
                  or Makie bbox       no Makie dep
```

| Stage | Input | Output | Dependencies |
|---|---|---|---|
| **Measure** | labels + font attrs + data→pixel transform | `Vector{Vec2f}` box sizes (px) + anchor pixels | TextMeasure *or* Makie bbox (dispatched on text type) |
| **Solve** | anchor pixels, box sizes, params | `Vector{Vec2f}` offsets + `BitVector` drop mask | **none** (pure Julia) |
| **Render** | original positions + solved offsets | text/box plots + connector lines | Makie |

The **solver is a standalone pure function** — no Makie types in or out, works entirely
in pixel space. It is the unit we test exhaustively and the unit a future reactive
wrapper re-calls on camera change.

## Measurement layer (pluggable)

Dispatches on label type:

- Plain `String` / `LaTeXString` → **TextMeasure** (`MakieBackend`): fast, no render pass,
  deterministic. Empirically floating-point-identical to Makie's rendered boxes at
  `px_per_unit = 1` (validated).
- `rich` / `RichText` → **Makie's `full_boundingbox(plot, :pixel)` after a cheap render**.
  Note: `Makie.text_bb` does **not** accept `RichText` (it `MethodError`s — plain strings
  only), so rich labels require one throwaway `text!` + `full_boundingbox` pass to measure.
  This is the one place we accept a Makie-internal dependency, isolated behind the
  measurement interface.

The solver and renderer don't care which measurer ran; they consume pixel box sizes.

**Two caveats the measurement layer must handle (validated):**

- **Font resolution.** `MakieBackend(font=...)` only accepts what `Makie.to_font` accepts
  (`String` / `Vector{String}` / `FTFont` / `automatic`) — it rejects `Symbol`s like
  `:bold`. The layer must resolve any `@inherit`/symbolic font to a `String`/`FTFont`
  before constructing the backend.
- **Glyph fallback.** TextMeasure sums glyph advances of the *resolved* font and does not
  replicate Makie's per-glyph font-fallback substitution. If the chosen font lacks the
  label's glyphs, TM's box diverges from Makie's. Defensive fallback: when the font
  doesn't cover the text, route through Makie's `text_bb` (plain) /
  `full_boundingbox` (rich) instead. Narrow real-world case (mismatched font/script).

**Implication for the pipeline:** the "render once" promise holds for the **plain/LaTeX**
path (render-free measurement). The **rich-text** path adds a cheap measure-render before
the final render.

**Follow-up:** [jowch/TextMeasure.jl#1](https://github.com/jowch/TextMeasure.jl/issues/1)
requests rich-text measurement in TextMeasure. If implemented, we register it as the
measurer for `RichText` and drop the Makie-internal fallback — a swap, not a rewrite.

## Public API

A Makie **recipe**, mirroring `text!`'s signature:

```julia
textrepel!(ax, xs, ys; text = labels, ...)       # or
textrepel!(ax, positions; text = labels, ...)     # positions::Vector{Point2}
```

A recipe (vs. a plain function) gives theme integration, `Cycled` colors, axis
attachment, and the reactive-ready seam — without wiring camera observables yet.

The recipe macro declares one positional (`positions`) and exposes `text` as a keyword
attribute, so `textrepel!(ax, positions; text = labels)` works for free (validated). The
`textrepel!(ax, xs, ys; text = labels)` convenience form additionally requires a
`Makie.convert_arguments` method turning `(xs, ys)` into `positions` — a small, standard
addition to implement during the build.

```julia
@recipe TextRepel (positions,) begin
    text          = nothing          # labels: Vector of String / rich / LaTeXString

    # ── Physics (pixel space, anisotropic) ──
    force         = (1.0, 1.0)        # label↔label repulsion (x, y)
    force_point   = (1.0, 1.0)        # label↔data-point repulsion
    force_pull    = (0.01, 0.01)      # spring back to anchor
    max_iter      = 2000
    only_move     = :both             # :both | :x | :y

    # ── Spacing / dropping policy ──
    box_padding   = 4.0               # px around each label box for the solver
    point_padding = 0.0               # px halo around data points
    max_overlaps  = Inf               # Inf = keep all; finite = drop (ggrepel-style)

    # ── Connector segments ──
    segments           = true
    min_segment_length = 5.0          # px; suppress connector if label barely moved
    segmentcolor       = :gray60
    linewidth          = 1.0

    # ── Background box (geom_label_repel style) ──
    background      = false
    backgroundcolor = (:white, 0.8)
    strokecolor     = :gray70
    strokewidth     = 0.0
    cornerradius    = 4.0

    # ── Text passthrough (inherited from theme) ──
    fontsize = @inherit fontsize
    font     = @inherit font
    color    = @inherit textcolor
    align    = (:center, :center)
end
```

Deliberate defaults: `max_overlaps = Inf` (keep all by default), and **no `seed`**
(the solver is deterministic, so there's nothing to seed).

## The solver

```julia
solve_repel(anchors::Vector{Point2f},     # data points, in PIXELS
            boxes::Vector{Vec2f},          # label width/height, in PIXELS
            params) -> (offsets::Vector{Vec2f}, dropped::BitVector)
```

Everything in pixels. Anchors via `register_projected_positions!` (data→pixel; validated
as exported and callable from inside `plot!`); box sizes from the measurement layer. A
label's current box = `anchor + offset ± size/2`.

Note (validated): `register_projected_positions!` yields **scene-local** pixels (origin at
the axis/scene lower-left, not figure-absolute). This is the correct frame for
`markerspace = :pixel` offsets, so the solver and the render hand-off stay consistent —
but unit tests outside a recipe should use `Makie.project(scene, :data, :pixel, p)` (the
one-shot escape hatch) and must not expect figure-absolute coordinates.

Algorithm (deterministic adjustText engine):

1. **Init** — each label centered on its anchor (offset = 0).
2. **Deterministic explosion pre-pass** — coincident/overlapping labels have a zero-length
   repulsion gradient and would never separate. Push them apart along *fixed* directions
   from a golden-angle spiral indexed by label order (replaces the upstreams' randomness →
   reproducible).
3. **Force loop** (`max_iter`, early-exit when max per-label movement < ε):
   - **Label↔label:** for each overlapping box pair, push along center-to-center vector,
     magnitude ∝ overlap extent, scaled by `force` (separate x/y).
   - **Label↔point:** push the box away from any data point within `point_padding`,
     scaled by `force_point`.
   - **Spring to anchor:** pull ∝ offset × `force_pull`, suppressed within a small
     threshold of the anchor (anti-oscillation).
   - **Step clamp:** cap per-iteration movement so the system can't explode.
   - `only_move` zeroes the x or y force component if axis-constrained.
4. **Dropping** — count residual overlaps per label; mark `dropped[i]` where overlaps >
   `max_overlaps`. Default `Inf` ⇒ nothing dropped.

**Rendering hand-off:** render `text!` at the original data `positions` with
`offset = offsets` (px) and `markerspace = :pixel`. Position stays in data space,
displacement in pixels — clean, and keeps the door open for reactivity. Dropped labels
are filtered out. When `background = true`, render boxed labels instead.

**Connectors:** anchor→label, clipped to the box edge (intersect the anchor→center ray
with the box rectangle), drawn only when offset magnitude > `min_segment_length`.

The loop touches only `Point2f`/`Vec2f`/`Rect2f` and plain floats — trivially
unit-testable in isolation.

## Testing strategy

Three tiers mirroring the layers:

1. **Solver** (the bulk) — pure-function property tests:
   - *No-overlap*: resolvable inputs → final boxes don't overlap (within tolerance).
   - *Determinism*: same input → byte-identical output across repeated runs.
   - *Anchor proximity*: labels end near their anchors.
   - *Axis constraint*: `only_move = :x` ⇒ zero y-displacement.
   - *Dropping policy*: finite `max_overlaps` drops the right count; `Inf` drops nothing.
   - *Stability*: degenerate cases (all coincident, single label, empty) don't NaN/blow up.
2. **Measurement layer** — TextMeasure `MakieBackend` boxes match Makie's rendered bbox
   (`full_boundingbox`/`text_bb`) within px tolerance; rich-text adapter returns finite,
   sane boxes.
3. **Integration (CairoMakie)** — end-to-end smoke tests that `textrepel!` runs and
   produces a plot; a small set of reference-image tests for visual regressions.

## Dependencies & compat

```toml
[deps]
Makie          # hard dep — the package IS a recipe
TextMeasure    # hard dep — measurement
# GeometryBasics types arrive via Makie re-export

[sources]
TextMeasure = {url = "https://github.com/jowch/TextMeasure.jl", rev = "main"}

# test-only (extras/targets):
CairoMakie, Test, ReferenceTests

[compat]
Makie = "0.24"
julia = "1.11"        # [sources] needs 1.11+; Makie 0.24 needs ≥1.10 anyway
```

`[sources]` is the modern way to depend on an **unregistered** package — CI resolves
TextMeasure from its git repo without registration, while developers `Pkg.dev` it
locally. When TextMeasure registers, swap `[sources]` for a normal `[compat]` entry.

## Out of scope (v1)

- Reactive re-solving on zoom/pan/resize (clean seams left in place for a later add-on).
- Discrete candidate-position / cartographic placement (future direction).
- Text rotation, per-glyph boxes, CJK/justification (TextMeasure limitations).
- Connector-crossing untangling (adjustText's experimental `prevent_crossings`).

## Future directions

- Reactive wrapper that re-calls `solve_repel` on camera/viewport changes.
- Discrete or hybrid (discrete init + force refinement) placement.
- Rich-text measurement via TextMeasure once
  [jowch/TextMeasure.jl#1](https://github.com/jowch/TextMeasure.jl/issues/1) lands.
