# MakieTextRepel.jl

A `ggrepel`/`adjustText`-style label-repel recipe for [Makie](https://docs.makie.org).
Automatically displaces overlapping text labels and draws connector lines back to
their data points.

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
