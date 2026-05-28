# Standalone spike: pluggable TextRepelAlgorithm for Makie's annotation! recipe.

using CairoMakie
using MakieTextRepel

points = [(-2.15, -0.19), (-1.66, 0.78), (-1.56, 0.87), (-0.97, -1.91),
          (-0.96, -0.25), (-0.79, 2.6),  (-0.74, 1.68),  (-0.56, -0.44),
          (-0.36, -0.63), (-0.32, 0.67), (-0.15, -1.11), (-0.07, 1.23),
          (0.3, 0.73),    (0.72, -1.48), (0.8, 1.12)]
fruit = ["Apple", "Banana", "Cherry", "Date", "Elderberry", "Fig",
         "Grape", "Honeydew", "Indian Fig", "Jackfruit", "Kiwi",
         "Lychee", "Mango", "Nectarine", "Orange"]
limits = (-3, 1.5, -3, 3)

fig = Figure(size = (1200, 400))

ax1 = Axis(fig[1, 1]; limits, title = "annotation! (default LabelRepel)")
scatter!(ax1, points); annotation!(ax1, points; text = fruit)

ax2 = Axis(fig[1, 2]; limits, title = "annotation! + TextRepelAlgorithm")
scatter!(ax2, points)
annotation!(ax2, points; text = fruit, algorithm = TextRepelAlgorithm())

ax3 = Axis(fig[1, 3]; limits, title = "textrepel! (recipe)")
scatter!(ax3, points); textrepel!(ax3, points; text = fruit)

hidedecorations!.([ax1, ax2, ax3])

out = joinpath(@__DIR__, "..", "test", "output", "annotation_spike.png")
mkpath(dirname(out))
save(out, fig)
println("wrote: ", out)

Makie.update_state_before_display!(fig.scene)
pl = filter(p -> p isa Annotation, ax2.scene.plots)[1]
offs = pl.offsets[]
println("n=", length(offs), "  max|offset|=", maximum(o -> hypot(o[1], o[2]), offs))
