using CairoMakie
using MakieTextRepel

@testset "recipe declaration" begin
    # the recipe type and functions exist
    @test isdefined(MakieTextRepel, :TextRepel)
    @test isa(textrepel!, Function)

    # (xs, ys) convenience form converts to a positions vector without error
    fig = Figure()
    ax = Axis(fig[1, 1])
    pl = textrepel!(ax, [1.0, 2.0, 3.0], [1.0, 2.0, 3.0];
                    text = ["a", "b", "c"])
    @test pl isa MakieTextRepel.TextRepel
end

@testset "textrepel end-to-end (plain text)" begin
    fig = Figure(size = (400, 400))
    ax = Axis(fig[1, 1])
    pts = Point2f[(1, 1), (1.01, 1.01), (1.02, 0.99)]   # nearly coincident
    pl = textrepel!(ax, pts; text = ["alpha", "beta", "gamma"])

    # force the scene to lay out so the recipe's compute graph runs
    Makie.update_state_before_display!(fig.scene)

    offs = pl.computed_offsets[]
    @test length(offs) == 3
    @test all(o -> all(isfinite, o), offs)
    @test maximum(norm, offs) > 0          # crowded labels actually moved
end

@testset "connectors render" begin
    fig = Figure(size = (400, 400))
    ax = Axis(fig[1, 1])
    pts = Point2f[(1, 1), (1.005, 1.005)]   # very close → labels must move far
    pl = textrepel!(ax, pts; text = ["overlapping one", "overlapping two"],
                    segments = true, min_segment_length = 1.0)
    Makie.update_state_before_display!(fig.scene)

    # a LineSegments child plot exists and has an even number of endpoints
    seg_plots = filter(c -> c isa LineSegments, pl.plots)
    @test length(seg_plots) == 1
    @test iseven(length(seg_plots[1][1][]))
end

@testset "background boxes" begin
    fig = Figure(size = (400, 400))
    ax = Axis(fig[1, 1])
    pl = textrepel!(ax, Point2f[(1, 1), (2, 2)]; text = ["one", "two"],
                    background = true)
    Makie.update_state_before_display!(fig.scene)

    # a Poly child plot exists when background = true
    poly_plots = filter(c -> c isa Poly, pl.plots)
    @test length(poly_plots) == 1
end

@testset "visual artifact renders" begin
    fig = Figure(size = (600, 400))
    ax = Axis(fig[1, 1]; title = "textrepel demo")
    pts = Point2f[(1, 1), (1.1, 1.05), (1.05, 0.9), (2, 2), (2.05, 2.02)]
    scatter!(ax, pts; color = :tomato)
    textrepel!(ax, pts; text = ["alpha", "beta", "gamma", "delta", "epsilon"],
               segments = true)
    out = joinpath(@__DIR__, "output", "demo.png")
    mkpath(dirname(out))
    save(out, fig)
    @test isfile(out) && filesize(out) > 0
end

@testset "axis limits track data, not pixel offsets" begin
    fig = Figure(); ax = Axis(fig[1, 1])
    pts = Point2f[(1, 1), (1.1, 1.05), (1.05, 0.9), (2, 2), (2.05, 2.02)]
    pl = textrepel!(ax, pts; text = ["a", "b", "c", "d", "e"])

    # The recipe's own data_limits must report the anchor extent only.
    dl = Makie.data_limits(pl)
    @test dl.widths[1] < 5
    @test dl.widths[2] < 5

    # And — crucially — the ACTUAL axis autolimits must follow it. The axis uses
    # boundingbox(scene, exclude) on the linear path, so this catches a leak that
    # a data_limits-only check would miss. Data spans ~x∈[1,2.05], y∈[0.9,2.02];
    # limits must stay near the data, not blow up to the pixel scale (~100+).
    reset_limits!(ax)
    lims = ax.finallimits[]
    @test lims.widths[1] < 5
    @test lims.widths[2] < 5
end
