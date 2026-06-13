# textrepel! quickstart — the standalone recipe.
#
# textrepel! takes the same positional data as scatter! (points, or x/y vectors)
# plus a `text` keyword, and places the labels so they don't overlap each other or
# sit on top of the markers, drawing a leader line back to each point.
#
# Run with:  julia --project=. examples/textrepel_quickstart.jl

using CairoMakie, MakieTextRepel

# A handful of points, with a couple sitting close enough to collide if labelled naively.
points = Point2f[(1.0, 1.0), (1.08, 1.04), (1.04, 0.92), (1.6, 1.5), (0.6, 1.7)]
labels = ["alpha", "beta", "gamma", "delta", "epsilon"]

fig = Figure(size = (900, 450), fontsize = 16)

# --- Left: the defaults ---------------------------------------------------
# Draw the labels first, then the scatter on top, so the leader lines tuck under
# the markers. Set `markersize` to the scatter marker size so the labels keep clear
# of the markers (it sets the marker-clearance distance for you).
ax1 = Axis(fig[1, 1]; title = "defaults")
textrepel!(ax1, points; text = labels, markersize = 12)
scatter!(ax1, points; markersize = 12, color = :tomato)

# --- Right: a few common tweaks -------------------------------------------
# background = true draws a box behind each label; only_move = :y locks horizontal
# position so labels move up/down only; point_padding sets the marker-clearance gap
# directly (here, instead of deriving it from markersize).
ax2 = Axis(fig[1, 2]; title = "boxed labels, vertical-only movement")
textrepel!(ax2, points; text = labels,
           point_padding = 8,
           only_move = :y,
           background = true,
           backgroundcolor = (:white, 0.85),
           strokecolor = :gray60, strokewidth = 0.5)
scatter!(ax2, points; markersize = 12, color = :tomato)

out = joinpath(@__DIR__, "..", "test", "output", "textrepel_quickstart.png")
mkpath(dirname(out))
save(out, fig)
println("wrote ", out)
