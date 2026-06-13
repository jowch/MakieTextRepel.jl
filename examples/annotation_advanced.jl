# TextRepelAlgorithm — the features beyond leader styling: per-label pinning,
# obstacle avoidance, over-capacity dropping, and reading solve diagnostics with
# solve_stats. (For leader styling — lines, arrows, colors — see annotation_styling.jl.)
#
# Run with:  julia --project=. examples/annotation_advanced.jl

using CairoMakie, MakieTextRepel

points = Point2f[(1.0, 1.0), (1.08, 1.04), (1.04, 0.92), (0.95, 1.06),
                 (1.6, 1.5), (0.6, 1.7), (1.5, 0.6), (0.7, 0.55)]
labels = ["alpha", "beta", "gamma", "delta", "epsilon", "zeta", "eta", "theta"]

fig = Figure(size = (1080, 400), fontsize = 15)

# All three axes share a fixed pixel size so the row stays even (and so the cramped
# panel below is genuinely over-capacity at a known size).
axkw = (; width = 300, height = 300)

# --- 1. Per-label pinning -------------------------------------------------
# annotation!'s first positional argument is a vector of label offsets (in
# :relative_pixel space): give an entry a finite value to PIN that label there,
# or Vec2f(NaN, NaN) to let the solver place it. Here we pin "alpha" and let the
# solver auto-place the other seven around it.
ax1 = Axis(fig[1, 1]; title = "pin one label, auto-place the rest", axkw...)
scatter!(ax1, points; markersize = 10, color = :tomato)
offs = fill(Vec2f(NaN, NaN), length(points))
offs[1] = Vec2f(48, 34)            # pin "alpha" 48 px right, 34 px up of its point
annotation!(ax1, offs, points; text = labels,
            algorithm = TextRepelAlgorithm(), labelspace = :relative_pixel)

# --- 2. Obstacle avoidance ------------------------------------------------
# Keep labels out of a region — e.g. where a legend or another plot sits.
# `obstacles` are Rect2f in pixel space; project a data-space rectangle to pixels
# with Makie.project once the axis scene is laid out.
ax2 = Axis(fig[1, 2]; title = "keep labels out of a region", axkw...)
scatter!(ax2, points; markersize = 10, color = :tomato)
Makie.update_state_before_display!(fig)        # establish the data→pixel transform
lo = Makie.project(ax2.scene, Point2f(1.12, 0.96))
hi = Makie.project(ax2.scene, Point2f(1.45, 1.18))
keepout = Rect2f(min.(lo, hi), abs.(hi .- lo))
poly!(ax2, keepout; space = :pixel, color = (:steelblue, 0.15))  # show the region
annotation!(ax2, points; text = labels,
            algorithm = TextRepelAlgorithm(obstacles = [keepout]))

# --- 3. Over-capacity dropping + solve_stats ------------------------------
# This panel packs more labels than fit: the solver keeps as many as it can place
# cleanly and drops the rest. solve_stats reports how the last solve went, including
# the drop count (printed below).
ax3 = Axis(fig[1, 3]; title = "cramped → some labels dropped", axkw...)
many = Point2f[(x, y) for x in range(0.15, 0.85, length = 6) for y in range(0.15, 0.85, length = 6)]
manylabels = ["station $(i)" for i in 1:length(many)]
scatter!(ax3, many; markersize = 5, color = :tomato)
alg3 = TextRepelAlgorithm()
annotation!(ax3, many; text = manylabels, algorithm = alg3)

out = joinpath(@__DIR__, "..", "test", "output", "annotation_advanced.png")
mkpath(dirname(out))
save(out, fig)
println("wrote ", out)

# solve_stats reflects the most recent solve for that algorithm instance, so read
# it after the figure has been built (save triggers the solve).
s = solve_stats(alg3)
println("panel 3 solve_stats: dropped=$(s.dropped) of $(length(many)), ",
        "overlaps=$(s.overlaps), crossings=$(s.crossings)")
