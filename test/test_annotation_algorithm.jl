using MakieTextRepel
using MakieTextRepel: TextRepelAlgorithm, RepelParams, solve_stats
using Test
using GeometryBasics

@testset "TextRepelAlgorithm — constructors" begin
    # Kwarg form, defaults
    alg = TextRepelAlgorithm()
    @test alg isa TextRepelAlgorithm
    @test alg.params isa RepelParams
    @test isempty(alg.obstacles)

    # Kwarg form, forwarded fields
    alg2 = TextRepelAlgorithm(force = (2.0, 2.0), only_move = :y)
    @test alg2.params.force == (2.0, 2.0)
    @test alg2.params.only_move == :y

    # Explicit form
    p = RepelParams(force = (3.0, 3.0))
    alg3 = TextRepelAlgorithm(p)
    @test alg3.params === p
    @test isempty(alg3.obstacles)

    # Explicit form with obstacles
    obs = [Rect2f(0, 0, 10, 10)]
    alg4 = TextRepelAlgorithm(p; obstacles = obs)
    @test alg4.obstacles === obs
end

@testset "TextRepelAlgorithm — unknown kwarg errors" begin
    @test_throws ArgumentError TextRepelAlgorithm(force_pul = (.02, .02))
    @test_throws ArgumentError TextRepelAlgorithm(not_a_field = 1)
end

@testset "TextRepelAlgorithm — bounds warning" begin
    @test_logs (:warn, r"bounds.*automatically") TextRepelAlgorithm(
        bounds = Rect2f(0, 0, 1, 1))
end

@testset "TextRepelAlgorithm — max_overlaps warning" begin
    @test_logs (:warn, r"max_overlaps") TextRepelAlgorithm(max_overlaps = 3)
    @test_logs (:warn, r"max_overlaps") TextRepelAlgorithm(
        RepelParams(max_overlaps = 3))
end
