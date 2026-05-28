# Generates the README hero image: plain text! (overlapping) vs textrepel!
# (resolved). Run with: julia --project=. examples/readme_example.jl
using CairoMakie, MakieTextRepel
using Random

Random.seed!(20260527)

# A tightly-clustered labelled scatter where naive placement collides badly.
n = 22
xs = randn(n) .* 0.7
ys = randn(n) .* 0.7
labels = ["node $(i)" for i in 1:n]

# Shared, roomy frame so repelled labels stay in view (and both panels match).
pad = 0.7
xl = (minimum(xs) - pad, maximum(xs) + pad)
yl = (minimum(ys) - pad, maximum(ys) + pad)

fig = Figure(size = (1000, 480), fontsize = 15)

ax1 = Axis(fig[1, 1]; title = "text! (overlapping)", aspect = 1)
scatter!(ax1, xs, ys; color = :tomato, markersize = 9)
text!(ax1, xs, ys; text = labels, align = (:left, :bottom), fontsize = 13)
xlims!(ax1, xl); ylims!(ax1, yl)

ax2 = Axis(fig[1, 2]; title = "textrepel! (resolved)", aspect = 1)
scatter!(ax2, xs, ys; color = :tomato, markersize = 9)
textrepel!(ax2, xs, ys; text = labels, fontsize = 13)
xlims!(ax2, xl); ylims!(ax2, yl)

mkpath(joinpath(@__DIR__, "..", "assets"))
out = joinpath(@__DIR__, "..", "assets", "example.png")
save(out, fig; px_per_unit = 2)
println("wrote ", abspath(out))
