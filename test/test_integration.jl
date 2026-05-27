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
