# Deep probe: recurse into Annotation children to find the connector Lines plot
# and inspect resolved color/linewidth/data.

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
ax1 = Axis(fig[1, 1]; limits, title = "default (auto color)")
scatter!(ax1, points)
ann1 = annotation!(ax1, points; text = fruit, algorithm = TextRepelAlgorithm())

ax2 = Axis(fig[1, 2]; limits, title = "explicit red, lw=1.5")
scatter!(ax2, points)
ann2 = annotation!(ax2, points; text = fruit, algorithm = TextRepelAlgorithm(),
                   color = :red, linewidth = 1.5)

ax3 = Axis(fig[1, 3]; limits, title = "gray60, lw=1.0 (textrepel-style)")
scatter!(ax3, points)
ann3 = annotation!(ax3, points; text = fruit, algorithm = TextRepelAlgorithm(),
                   color = :gray60, linewidth = 1.0)

hidedecorations!.([ax1, ax2, ax3])
Makie.update_state_before_display!(fig.scene)

walk_plots(p, depth=0) = begin
    indent = "  " ^ depth
    nm = typeof(p).name.name
    println(indent, "- ", nm, "  (", typeof(p), ")")
    for c in p.plots
        walk_plots(c, depth + 1)
    end
end

for (i, ann) in enumerate((ann1, ann2, ann3))
    println("\n=== panel $i ===")
    println("ann.color[]      = ", ann.color[])
    println("ann.linewidth[]  = ", ann.linewidth[])
    println("ann.shrink[]     = ", ann.shrink[])
    println("ann.style[]      = ", ann.style[])
    println("ann.visible[]    = ", ann.visible[])
    offs = ann.offsets[]
    finite = filter(o -> all(isfinite, o), offs)
    println("n_offsets=", length(offs), "  n_nonzero=",
            count(o -> hypot(o[1], o[2]) > 0.5, finite),
            "  max|offset|=", isempty(finite) ? 0.0 : maximum(o -> hypot(o[1], o[2]), finite))
    println("plot tree:")
    walk_plots(ann)

    # find any Lines descendant
    function find_lines(p, out=[])
        for c in p.plots
            if c isa Lines
                push!(out, c)
            else
                find_lines(c, out)
            end
        end
        out
    end
    lns = find_lines(ann)
    println("Lines descendants: ", length(lns))
    for (j, ln) in enumerate(lns[1:min(3, end)])
        # try to read positions and color
        pos = try ln[1][] catch e; "ERR: $e" end
        col = try ln.color[] catch e; "ERR: $e" end
        lw  = try ln.linewidth[] catch e; "ERR: $e" end
        vis = try ln.visible[] catch e; "ERR: $e" end
        sp  = try ln.space[] catch e; "ERR: $e" end
        println("  Lines[$j] color=", col, "  lw=", lw, "  vis=", vis,
                "  space=", sp, "  n_pts=", length(pos isa AbstractVector ? pos : []))
        if pos isa AbstractVector && !isempty(pos)
            println("    first=", pos[1], "  last=", pos[end])
        end
    end

    # also inspect plotspecs
    specs = try ann.plotspecs[] catch e; nothing end
    if specs !== nothing
        println("plotspecs count: ", length(specs))
        # show the first few :Lines specs
        line_specs = filter(s -> hasproperty(s, :type) ? s.type == :Lines : false, specs)
        # PlotSpec API: each has .plottype and .kwargs
        for (k, s) in enumerate(specs[1:min(4, end)])
            println("  spec[$k] ", s)
        end
    end
end

out = joinpath(@__DIR__, "..", "test", "output", "probe_annotation_internals.png")
save(out, fig)
println("\nwrote: ", out)
