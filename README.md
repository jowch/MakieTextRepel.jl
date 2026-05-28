# MakieTextRepel.jl

A `ggrepel`/`adjustText`-style label-repel recipe for [Makie](https://docs.makie.org).
Automatically displaces overlapping text labels and draws connector lines back to
their data points.

![text! labels overlapping versus textrepel! labels resolved with connector lines](assets/example.png)

*Left: plain `text!` labels collide. Right: `textrepel!` separates them and draws
connectors back to each point. (Reproduce with [`examples/readme_example.jl`](examples/readme_example.jl).)*

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

Key attributes: `force`, `force_point`, `force_pull` (anisotropic `(x, y)` tuples),
`only_move` (`:both`/`:x`/`:y`), `max_overlaps` (`Inf` keeps all labels; finite drops
crowded ones), `background` (boxed labels), `segments`/`segmentcolor`/`linewidth`
(connectors).

## How it works

Three layers: **measure** (pixel box sizes via TextMeasure.jl; rich text via Makie),
**solve** (a deterministic force-directed solver in pixel space), **render** (text +
optional boxes + connectors). Output is deterministic — same data, same figure.

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
