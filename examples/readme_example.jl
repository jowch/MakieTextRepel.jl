# Generates the README hero image: plain text! (overlapping) vs textrepel!
# (resolved) vs annotation! driven by TextRepelAlgorithm.
# Run with: julia --project=. examples/readme_example.jl
using CairoMakie, MakieTextRepel
using Random

Random.seed!(20260527)

# A tightly-clustered labelled scatter where naive placement collides badly.
n = 22
xs = randn(n) .* 0.7
ys = randn(n) .* 0.7
labels = ["node $(i)" for i in 1:n]
points = Point2f.(xs, ys)

fig = Figure(size = (1480, 480), fontsize = 15)

ax1 = Axis(fig[1, 1]; title = "text! (overlapping)", aspect = 1)
scatter!(ax1, xs, ys; color = :tomato, markersize = 9)
text!(ax1, xs, ys; text = labels, align = (:left, :bottom), fontsize = 13)

# Draw the labels (and their connectors) first, then the scatter on top, so the
# leader lines tuck underneath the markers instead of crossing over them.
ax2 = Axis(fig[1, 2]; title = "textrepel! (resolved)", aspect = 1)
textrepel!(ax2, xs, ys; text = labels, fontsize = 13)
scatter!(ax2, xs, ys; color = :tomato, markersize = 9)

# Same solver, plugged into Makie.annotation! via the algorithm hook — plain
# leader lines (no arrow heads) via annotation!'s own connector styling.
ax3 = Axis(fig[1, 3]; title = "annotation! + TextRepelAlgorithm", aspect = 1)
scatter!(ax3, xs, ys; color = :tomato, markersize = 9)
annotation!(ax3, points; text = labels, fontsize = 13,
            style = Makie.Ann.Styles.Line(),
            algorithm = TextRepelAlgorithm())

mkpath(joinpath(@__DIR__, "..", "assets"))
out = joinpath(@__DIR__, "..", "assets", "example.png")
save(out, fig; px_per_unit = 2)
println("wrote ", abspath(out))
