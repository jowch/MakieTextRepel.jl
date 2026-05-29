# Generates the README hero image: a 3-panel comparison on one clustered dataset.
#   text! (overlapping)  →  textrepel! (resolved)  →  annotation! + TextRepelAlgorithm
# The last two share the same solver (via solve_cluster), so their layouts match —
# that's the point of issue #12: one solver, two surfaces.
# Run with: julia --project=. examples/readme_example.jl
using CairoMakie, MakieTextRepel
using Random

Random.seed!(20260527)

# A few small "knots" of near-coincident points plus a scattered field. The knots
# make naive placement collide; the resolvers fan them out with leader lines while
# the sparse singles stay readable. Small knots (2-3 pts) keep the leaders tidy.
knot_centers = [(-0.7, 0.55), (0.75, -0.45), (0.05, 0.9)]
knot_counts  = [3, 3, 2]
xs = Float64[]
ys = Float64[]
for (c, k) in zip(knot_centers, knot_counts)
    for _ in 1:k
        push!(xs, c[1] + 0.03 * randn())
        push!(ys, c[2] + 0.03 * randn())
    end
end
for _ in 1:14                       # scattered singles filling the field
    push!(xs, 0.85 * randn())
    push!(ys, 0.85 * randn())
end
n = length(xs)                      # 3 + 3 + 2 + 14 = 22
labels = ["node $(i)" for i in 1:n]
points = Point2f.(xs, ys)

fig = Figure(size = (1200, 400), fontsize = 15)

# Left: plain text! — labels sit on the points and collide at the knots.
ax1 = Axis(fig[1, 1]; title = "text! (overlapping)", aspect = 1)
scatter!(ax1, xs, ys; color = :tomato, markersize = 9)
text!(ax1, xs, ys; text = labels, align = (:left, :bottom), fontsize = 13)

# Middle: textrepel!. Draw the labels (and connectors) first, then the scatter on
# top, so the leader lines tuck underneath the markers.
ax2 = Axis(fig[1, 2]; title = "textrepel! (resolved)", aspect = 1)
textrepel!(ax2, xs, ys; text = labels, fontsize = 13)
scatter!(ax2, xs, ys; color = :tomato, markersize = 9)

# Right: the same solver, plugged into Makie.annotation! via the algorithm hook —
# plain leader lines (no arrow heads) via annotation!'s own styling. Its layout
# matches the textrepel! panel: both go through solve_cluster.
ax3 = Axis(fig[1, 3]; title = "annotation! + TextRepelAlgorithm", aspect = 1)
scatter!(ax3, xs, ys; color = :tomato, markersize = 9)
annotation!(ax3, points; text = labels, fontsize = 13,
            style = Makie.Ann.Styles.Line(),
            algorithm = TextRepelAlgorithm())

mkpath(joinpath(@__DIR__, "..", "assets"))
out = joinpath(@__DIR__, "..", "assets", "example.png")
save(out, fig; px_per_unit = 2)
println("wrote ", abspath(out))
