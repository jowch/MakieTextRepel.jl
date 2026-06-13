# TextRepelAlgorithm — repelled labels with annotation!'s styling.
#
# TextRepelAlgorithm plugs the same solver into Makie.annotation!, so you get
# MakieTextRepel's placement underneath annotation!'s leader styling — plain lines,
# arrow heads, custom paths. Pass it as the `algorithm` keyword; style the leaders
# with annotation!'s own `style`/`color`/`linewidth` keywords.
#
# Run with:  julia --project=. examples/annotation_styling.jl

using CairoMakie, MakieTextRepel

points = Point2f[(1.0, 1.0), (1.08, 1.04), (1.04, 0.92), (1.6, 1.5), (0.6, 1.7)]
labels = ["alpha", "beta", "gamma", "delta", "epsilon"]

fig = Figure(size = (1350, 450), fontsize = 16)

# --- Left: default annotation! styling ------------------------------------
ax1 = Axis(fig[1, 1]; title = "default style")
scatter!(ax1, points; markersize = 12, color = :tomato)
annotation!(ax1, points; text = labels, algorithm = TextRepelAlgorithm())

# --- Middle: plain lines, recoloured --------------------------------------
ax2 = Axis(fig[1, 2]; title = "Ann.Styles.Line()")
scatter!(ax2, points; markersize = 12, color = :tomato)
annotation!(ax2, points; text = labels,
            algorithm = TextRepelAlgorithm(),
            style = Makie.Ann.Styles.Line(), color = :steelblue, linewidth = 1.5)

# --- Right: arrow heads + solver options ----------------------------------
# annotation! has no `markersize` convenience knob, so pass the marker-clearance gap
# to the algorithm directly via `point_padding`. Solver options (only_move,
# box_padding, point_padding, …) are TextRepelAlgorithm keywords.
ax3 = Axis(fig[1, 3]; title = "arrow heads, point_padding = 8")
scatter!(ax3, points; markersize = 12, color = :tomato)
annotation!(ax3, points; text = labels,
            algorithm = TextRepelAlgorithm(point_padding = 8),
            style = Makie.Ann.Styles.LineArrow())

out = joinpath(@__DIR__, "..", "test", "output", "annotation_styling.png")
mkpath(dirname(out))
save(out, fig)
println("wrote ", out)
