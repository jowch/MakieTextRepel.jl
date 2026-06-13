# Generates the README hero image: a 3-panel comparison on one clustered dataset.
#   text! (overlapping)  →  textrepel! (resolved)  →  annotation! + TextRepelAlgorithm
# textrepel! and TextRepelAlgorithm use the same solver, so their layouts match —
# one solver, two surfaces.
# Run with: julia --project=. examples/readme_example.jl
using CairoMakie, MakieTextRepel
using Random

# A few small "knots" of near-coincident points plus a scattered field. The knots
# make naive placement collide; the resolvers fan them out with leader lines while
# the sparse singles stay readable. Small knots (2-3 pts) keep the leaders tidy.
# NOTE: the exact RNG sequence is load-bearing — test/test_integration.jl
# reconstructs this dataset verbatim to compare against the committed image, so the
# seed and the x-then-y draw order must not change.
function hero_dataset()
    Random.seed!(20260527)
    knot_centers = [(-0.7, 0.55), (0.75, -0.45), (0.05, 0.9)]
    knot_counts  = [3, 3, 2]
    xs = Float64[]; ys = Float64[]
    for (c, k) in zip(knot_centers, knot_counts), _ in 1:k
        push!(xs, c[1] + 0.03 * randn()); push!(ys, c[2] + 0.03 * randn())
    end
    for _ in 1:14
        push!(xs, 0.85 * randn()); push!(ys, 0.85 * randn())
    end
    labels = ["node $(i)" for i in 1:length(xs)]
    return xs, ys, labels
end

xs, ys, labels = hero_dataset()     # 3 + 3 + 2 + 14 = 22 points
points = Point2f.(xs, ys)

# 1650×550, three aspect=1 panels. The canvas needs enough room for every label to
# clear its marker without being dropped; this size leaves headroom for all 22.
fig = Figure(size = (1650, 550), fontsize = 19)

# Left: plain text! — labels sit on the points and collide at the knots.
ax1 = Axis(fig[1, 1]; title = "text! (overlapping)", aspect = 1)
scatter!(ax1, xs, ys; color = :tomato, markersize = 12)
text!(ax1, xs, ys; text = labels, align = (:left, :bottom), fontsize = 17)

# Middle: textrepel!. Draw the labels (and connectors) first, then the scatter on
# top, so the leader lines tuck underneath the markers.
ax2 = Axis(fig[1, 2]; title = "textrepel! (resolved)", aspect = 1)
# markersize = 12 matches the scatter marker below, so textrepel! clears the markers.
textrepel!(ax2, xs, ys; text = labels, fontsize = 17, markersize = 12)
scatter!(ax2, xs, ys; color = :tomato, markersize = 12)

# Right: the same solver, plugged into Makie.annotation! via the `algorithm` keyword,
# with plain leader lines from annotation!'s own styling. Its layout matches the
# textrepel! panel because both use the same solver.
ax3 = Axis(fig[1, 3]; title = "annotation! + TextRepelAlgorithm", aspect = 1)
scatter!(ax3, xs, ys; color = :tomato, markersize = 12)
# annotation! has no `markersize` convenience knob, so set point_padding directly to
# the same clearance textrepel!'s markersize=12 derives: marker radius 6 + 0.5px gap.
annotation!(ax3, points; text = labels, fontsize = 17,
            style = Makie.Ann.Styles.Line(),
            algorithm = TextRepelAlgorithm(; point_padding = 12 / 2 + 0.5))

mkpath(joinpath(@__DIR__, "..", "assets"))
out = joinpath(@__DIR__, "..", "assets", "example.png")
save(out, fig; px_per_unit = 2)
println("wrote ", abspath(out))
