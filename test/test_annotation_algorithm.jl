using MakieTextRepel
using MakieTextRepel: TextRepelAlgorithm, RepelParams, solve_stats
using Test
using GeometryBasics
using LinearAlgebra
import Makie

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

@testset "solve_stats — initial state" begin
    alg = TextRepelAlgorithm()
    s = solve_stats(alg)
    @test s isa NamedTuple
    @test propertynames(s) == (:iter, :residual)
    @test s.iter == 0
    @test s.residual === 0f0
end

@testset "dispatch — basic happy path" begin
    # Three labels with NaN textpositions_offset (auto mode), all centered text.
    n = 3
    offsets              = [Vec2f(0, 0) for _ in 1:n]
    textpositions        = [Point2f(0, 0), Point2f(100, 0), Point2f(200, 0)]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    # Centered bboxes: origin = textposition - widths/2.
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)

    alg = TextRepelAlgorithm()
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    # All offsets populated, not all zero.
    @test all(isfinite, [o[1] for o in offsets])
    @test all(isfinite, [o[2] for o in offsets])
    @test any(o -> norm(o) > 0, offsets)

    # Diagnostics populated.
    s = solve_stats(alg)
    @test s.iter > 0
    @test s.residual >= 0
end

@testset "dispatch — n=0 early return" begin
    offsets = Vec2f[]
    Makie.calculate_best_offsets!(TextRepelAlgorithm(),
                                  offsets, Point2f[], Point2f[],
                                  Rect2f[], Rect2f(0, 0, 100, 100);
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)
    @test isempty(offsets)  # untouched
end

@testset "dispatch — all-pinned bypass" begin
    n = 3
    offsets       = [Vec2f(0, 0) for _ in 1:n]
    textpositions = [Point2f(0, 0), Point2f(100, 0), Point2f(200, 0)]
    # All finite => manual mode.
    textpositions_offset = [Point2f(p[1] + 10, p[2] + 20) for p in textpositions]
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)

    alg = TextRepelAlgorithm()
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    for i in 1:n
        @test offsets[i][1] ≈ 10f0
        @test offsets[i][2] ≈ 20f0
    end
    @test solve_stats(alg) == (iter = 0, residual = 0f0)
end
