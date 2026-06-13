# MakieTextRepel.jl

A `ggrepel`/`adjustText`-style label-repel recipe for [Makie](https://docs.makie.org).
Automatically displaces overlapping text labels and draws connector lines back to
their data points.

![Three panels: text! labels overlapping, textrepel! labels resolved with connector lines, and annotation! labels placed by TextRepelAlgorithm](assets/example.png)

*Left: plain `text!` labels collide. Middle: `textrepel!` separates them and draws
connectors back to each point. Right: the same solver driving `Makie.annotation!`
through `TextRepelAlgorithm`. (Reproduce with [`examples/readme_example.jl`](examples/readme_example.jl).)*

## Installation

MakieTextRepel depends on the (currently unregistered) TextMeasure.jl. Until both are
registered:

```julia
using Pkg
Pkg.add(url="https://github.com/jowch/TextMeasure.jl")
Pkg.add(url="https://github.com/jowch/MakieTextRepel.jl")
```

## Usage

```julia
using CairoMakie, MakieTextRepel

fig = Figure()
ax = Axis(fig[1, 1])
pts = Point2f[(1, 1), (1.1, 1.05), (1.05, 0.9)]
scatter!(ax, pts)
textrepel!(ax, pts; text = ["alpha", "beta", "gamma"])
fig
```

Key attributes: `only_move` (`:both`/`:x`/`:y`, constrains which axis labels may
move), `box_padding` (label breathing room), `point_padding` (**marker clearance** —
the minimum gap kept between every scatter marker and the nearest label text edge,
enforced after layout for a label's own *and* its neighbours' markers; default 5 px),
`markersize` (convenience: set it to your `scatter!` marker size and `point_padding`
is derived as `markersize/2 + 0.5`), `min_segment_length` (connector visibility
cutoff), `background` (boxed labels), `segments`/`segmentcolor`/`linewidth`
(connectors). The default solver guarantees zero label overlap, keeps labels clear of
the markers, and automatically drops labels when a scene is over-capacity.

> The force-tuning attributes `force`, `force_point`, `force_pull`, `max_iter`, and
> `max_overlaps` are **inert under the default `ProjectionSolver`** (it has no force
> loop and resolves overlaps geometrically); they affect only the non-default in-tree
> force solver.

## How it works

Three layers:

- **measure** — every label's box is sized from its *true* rendered extent via
  [TextMeasure.jl](https://github.com/jowch/TextMeasure.jl), render-free (no `Scene`
  is allocated; plain strings, LaTeX, and Makie rich text are all supported). Because
  placement works from real glyph metrics rather than character-count estimates,
  overlap removal and marker clearance are computed against the actual text box — not
  an approximation.
- **solve** — a deterministic, zero-overlap projection solver in pixel space: discrete
  Imhof side-selection → crossing repair → Dykstra constraint-projection legalization,
  with every data anchor treated as a keep-out so labels never sit under their markers.
- **render** — text + optional boxes + connectors.

Output is deterministic — same data, same figure.

## Two surfaces

MakieTextRepel exposes two ways to get repelled labels into a plot:

### `textrepel!`

A standalone Makie recipe with the full feature set: zero-overlap
projection solver, automatic over-capacity dropping, background boxes,
and pixel-space connector trimming.

```julia
using MakieTextRepel
scatter!(ax, points)
textrepel!(ax, points; text = labels, only_move = :y)
```

Use when you need any of those features, or for the default ggrepel /
adjustText workflow.

### `TextRepelAlgorithm`

An algorithm plug-in for `Makie.annotation!`. Reuses the same
zero-overlap solver underneath `annotation!`'s styling
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
