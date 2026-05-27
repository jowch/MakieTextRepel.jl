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
