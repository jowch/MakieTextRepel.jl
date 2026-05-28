using CairoMakie
using MakieTextRepel
include(joinpath(@__DIR__, "..", "src", "annotation_algorithm.jl"))

points = [(-2.15, -0.19), (-1.66, 0.78), (-1.56, 0.87), (-0.97, -1.91),
          (-0.96, -0.25), (-0.79, 2.6),  (-0.74, 1.68),  (-0.56, -0.44),
          (-0.36, -0.63), (-0.32, 0.67), (-0.15, -1.11), (-0.07, 1.23),
          (0.3, 0.73),    (0.72, -1.48), (0.8, 1.12)]
fruit = ["Apple", "Banana", "Cherry", "Date", "Elderberry", "Fig",
         "Grape", "Honeydew", "Indian Fig", "Jackfruit", "Kiwi",
         "Lychee", "Mango", "Nectarine", "Orange"]
limits = (-3, 1.5, -3, 3)

fig = Figure(size = (1200, 400))

ax1 = Axis(fig[1, 1]; limits, title = "default style (automatic)")
scatter!(ax1, points)
ann1 = annotation!(ax1, points; text = fruit,
                   algorithm = TextRepelAlgorithm())

ax2 = Axis(fig[1, 2]; limits, title = "explicit Ann.Styles.Line, red")
scatter!(ax2, points)
ann2 = annotation!(ax2, points; text = fruit,
                   algorithm = TextRepelAlgorithm(),
                   style = Makie.Ann.Styles.Line(),
                   color = :red, linewidth = 1.5)

ax3 = Axis(fig[1, 3]; limits, title = "default + huge offsets")
scatter!(ax3, points)
# Force big offsets via larger force values
ann3 = annotation!(ax3, points; text = fruit,
                   algorithm = TextRepelAlgorithm(force = (5.0, 5.0)),
                   color = :blue)

hidedecorations!.([ax1, ax2, ax3])

Makie.update_state_before_display!(fig.scene)
for (i, ann) in enumerate((ann1, ann2, ann3))
    line_plots = filter(p -> p isa Lines, ann.plots)
    text_plots = filter(p -> p isa Makie.Text, ann.plots)
    println("panel $i: ", length(line_plots), " Lines, ",
            length(text_plots), " Text, all children=",
            length(ann.plots), " (", [typeof(p).name.name for p in ann.plots], ")")
end

out = joinpath(@__DIR__, "..", "test", "output", "annotation_lines_probe.png")
mkpath(dirname(out))
save(out, fig)
println("wrote: ", out)
