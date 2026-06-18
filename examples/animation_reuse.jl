# examples/animation_reuse.jl
# Demonstrates the measure-once / layout-many property (#25): in an animation the label
# text is constant, so positions update every frame and the labels are re-placed —
# WITHOUT re-measuring. The reuse is structural: the recipe keys text measurement on
# `text`/`fontsize`/`font` only (a separate compute node from the solve), so a per-frame
# position change re-solves placement but never re-runs measurement. (The faithful unit
# proof of this lives in the test suite — "measurement reuse across solve-only updates".
# Note: comparing `computed_sizes` object identity does NOT prove reuse here, because
# Makie's ComputeGraph deduplicates equal-valued node outputs.)
using CairoMakie
using MakieTextRepel

fig = Figure(size = (500, 500))
ax = Axis(fig[1, 1], limits = (0, 1, 0, 1), title = "textrepel! animation (measurements reused)")

labels = ["alpha", "beta", "gamma", "delta", "epsilon"]
pos = Observable(Point2f[(0.5, 0.5), (0.52, 0.51), (0.48, 0.49), (0.51, 0.47), (0.49, 0.53)])
scatter!(ax, pos; markersize = 8)
pl = textrepel!(ax, pos; text = labels, markersize = 8)

outfile = joinpath(@__DIR__, "animation_reuse.mp4")
record(fig, outfile, 1:90; framerate = 30) do frame
    t = frame / 90 * 2pi
    # Anchors drift on small circles; text never changes ⇒ no re-measurement.
    pos[] = Point2f[(0.5 + 0.12cos(t + i), 0.5 + 0.12sin(t + i)) for i in 1:length(labels)]
end
@info "wrote $outfile"
