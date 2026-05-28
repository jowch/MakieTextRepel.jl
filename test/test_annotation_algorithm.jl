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

@testset "dispatch — per-label pinning (mixed mode)" begin
    n = 3
    textpositions = [Point2f(0, 0), Point2f(100, 0), Point2f(200, 0)]
    # Pin label 2 at (110, 30); labels 1 and 3 auto.
    textpositions_offset = [Point2f(NaN, NaN), Point2f(110, 30), Point2f(NaN, NaN)]
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)
    offsets  = [Vec2f(0, 0) for _ in 1:n]

    alg = TextRepelAlgorithm()
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    # Pinned label: offset is exactly the pin (relative to its textposition).
    @test offsets[2][1] ≈ 10f0   # 110 - 100
    @test offsets[2][2] ≈ 30f0

    # Auto-placed labels: not zero (solver ran).
    @test offsets[1] != Vec2f(0, 0)
    @test offsets[3] != Vec2f(0, 0)

    # Pinned bbox acts as an obstacle for auto-placed labels:
    # neither auto label's rendered bbox overlaps the pinned label's
    # rendered bbox.
    function _rendered(i)
        Rect2f(text_bbs[i].origin[1] + offsets[i][1],
               text_bbs[i].origin[2] + offsets[i][2],
               text_bbs[i].widths[1],
               text_bbs[i].widths[2])
    end
    pinned_rect = _rendered(2)
    for i in (1, 3)
        r = _rendered(i)
        overlaps_x = !(r.origin[1] + r.widths[1] <= pinned_rect.origin[1] ||
                       pinned_rect.origin[1] + pinned_rect.widths[1] <= r.origin[1])
        overlaps_y = !(r.origin[2] + r.widths[2] <= pinned_rect.origin[2] ||
                       pinned_rect.origin[2] + pinned_rect.widths[2] <= r.origin[2])
        @test !(overlaps_x && overlaps_y)
    end
end

@testset "dispatch — pin × only_move bypasses constraint (D6)" begin
    textpositions = [Point2f(0, 0), Point2f(100, 0)]
    # Pin label 2 with non-zero x component; alg uses only_move = :y.
    textpositions_offset = [Point2f(NaN, NaN), Point2f(120, 30)]
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)
    offsets  = [Vec2f(0, 0) for _ in 1:2]

    alg = TextRepelAlgorithm(only_move = :y)
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    # The pinned label keeps its x component (20 px) despite only_move = :y.
    @test offsets[2][1] ≈ 20f0
    @test offsets[2][2] ≈ 30f0
end

@testset "dispatch — alignment-correct anchor (D5)" begin
    # Same textposition, same widths, two different alignments.
    textpositions        = [Point2f(50, 50)]
    textpositions_offset = [Point2f(NaN, NaN)]
    viewport             = Rect2f(0, 0, 500, 500)

    # Center-aligned bbox: origin = textposition - widths/2.
    text_bbs_center = [Rect2f(30, 45, 40, 10)]
    offsets_center  = [Vec2f(0, 0)]
    Makie.calculate_best_offsets!(TextRepelAlgorithm(), offsets_center,
        textpositions, textpositions_offset, text_bbs_center, viewport;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)

    # Left-aligned bbox: origin = textposition (bbox extends right).
    text_bbs_left = [Rect2f(50, 45, 40, 10)]
    offsets_left  = [Vec2f(0, 0)]
    Makie.calculate_best_offsets!(TextRepelAlgorithm(), offsets_left,
        textpositions, textpositions_offset, text_bbs_left, viewport;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)

    # With the alignment fix, the left-aligned solve's offset is shifted
    # right by approximately widths/2 = 20px relative to the centered one
    # (the alignment pre-bias is added to init_state). Without the fix,
    # both solves are anchored identically and the delta would be ~0.
    delta = offsets_left[1] - offsets_center[1]
    @test isapprox(delta[1], 20f0, atol = 5.0)
    @test isapprox(delta[2], 0f0,  atol = 5.0)
end
