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
    @test Set(propertynames(s)) ==
          Set((:iter, :residual, :overlaps, :point_overlaps, :mean_leader, :crossings, :dropped))
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
    @test s.iter >= 0
    @test s.overlaps == 0
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
    @test solve_stats(alg) == (; overlaps = 0, point_overlaps = 0, mean_leader = 0f0,
                                 crossings = 0, iter = 0, residual = 0f0, dropped = 0)
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

@testset "dispatch — alignment is incorporated (D5)" begin
    # Same textposition and widths, two text_bb.origin conventions
    # (centered vs left-aligned). The wrapper must detect the alignment
    # and write different offsets for the two cases — otherwise annotation!
    # would render both bboxes shifted by `widths/2` relative to each
    # other in screen space. The substantive geometric check (rendered
    # bbox stays inside the viewport when the solver clamps it inside)
    # lives in "non-centered bbox near wall stays in bounds" below; this
    # test just confirms the alignment is detected. (D5 is also implicitly
    # exercised by every centered-bbox test, where align_bias is zero.)
    textpositions        = [Point2f(50, 50)]
    textpositions_offset = [Point2f(NaN, NaN)]
    viewport             = Rect2f(0, 0, 500, 500)

    text_bbs_center = [Rect2f(30, 45, 40, 10)]
    offsets_center  = [Vec2f(0, 0)]
    Makie.calculate_best_offsets!(TextRepelAlgorithm(), offsets_center,
        textpositions, textpositions_offset, text_bbs_center, viewport;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)

    text_bbs_left = [Rect2f(50, 45, 40, 10)]
    offsets_left  = [Vec2f(0, 0)]
    Makie.calculate_best_offsets!(TextRepelAlgorithm(), offsets_left,
        textpositions, textpositions_offset, text_bbs_left, viewport;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)

    @test offsets_left[1] != offsets_center[1]
    # Sanity: centered solve moves the bbox off the anchor to an Imhof slot
    # (shorter leader under ProjectionSolver).
    @test norm(offsets_center[1]) > 0
end

@testset "dispatch — non-centered bbox near wall stays in bounds" begin
    # Left-aligned label near the right wall. The solver clamps the bbox
    # inside bounds in solver-space; the alignment translation must carry
    # that through so the *rendered* bbox is also inside bounds. Pre-fix,
    # the writeback double-shifted by align_bias and the label escaped the
    # viewport by ~widths/2.
    textpositions        = [Point2f(480, 250)]
    textpositions_offset = [Point2f(NaN, NaN)]
    text_bbs             = [Rect2f(480, 245, 80, 10)]  # left-aligned, extends right past 500
    viewport             = Rect2f(0, 0, 500, 500)
    offsets              = [Vec2f(0, 0)]

    Makie.calculate_best_offsets!(TextRepelAlgorithm(), offsets,
        textpositions, textpositions_offset, text_bbs, viewport;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)

    rx = text_bbs[1].origin[1] + offsets[1][1]
    ry = text_bbs[1].origin[2] + offsets[1][2]
    @test rx                          >= viewport.origin[1]
    @test rx + text_bbs[1].widths[1]  <= viewport.origin[1] + viewport.widths[1]
    @test ry                          >= viewport.origin[2]
    @test ry + text_bbs[1].widths[2]  <= viewport.origin[2] + viewport.widths[2]
end

@testset "dispatch — warm-start when reset == false" begin
    n = 3
    textpositions = [Point2f(0, 0), Point2f(100, 0), Point2f(200, 0)]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 2000, 2000)   # large viewport; (500, 500) is far inside

    # Modest max_iter so neither solve converges; step_max small so neither
    # can fully traverse from (500, 500) to equilibrium.
    alg = TextRepelAlgorithm(max_iter = 10, step_max = 5.0)

    # Warm-start: pre-populated extreme offsets, reset = false.
    offsets_warm = [Vec2f(500, 500) for _ in 1:n]
    Makie.calculate_best_offsets!(alg, offsets_warm,
        textpositions, textpositions_offset, text_bbs, bbox;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = false)

    # Fresh-start: same pre-populated input, but reset = true (solver
    # discards offsets and uses init_offsets / align_bias instead).
    offsets_fresh = [Vec2f(500, 500) for _ in 1:n]
    Makie.calculate_best_offsets!(alg, offsets_fresh,
        textpositions, textpositions_offset, text_bbs, bbox;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)

    # Warm-start preserves position relative to fresh-start. Each
    # warm-start offset is closer to (500, 500) than its fresh counterpart.
    for i in 1:n
        @test norm(offsets_warm[i] - Vec2f(500, 500)) <
              norm(offsets_fresh[i] - Vec2f(500, 500))
    end
end

@testset "dispatch — warm-start preserves own-anchor invariant (R3)" begin
    # After equilibrium under reset=true the anchor must not lie strictly
    # inside each label's bbox; a subsequent reset=false solve should
    # maintain that invariant — warm-start mustn't degenerate the layout.
    # Note: ProjectionSolver places labels tangent to the anchor with
    # the default point_padding = 0 (the anchor may sit exactly ON the box
    # edge; the connector is then suppressed by min_segment_length), so the
    # invariant is "anchor is not strictly INSIDE the box interior."
    n = 4
    textpositions = [Point2f(50i, 50) for i in 1:n]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(p[1] - 15, p[2] - 5, 30, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)

    alg = TextRepelAlgorithm(max_iter = 500)
    offsets = [Vec2f(0, 0) for _ in 1:n]

    # Solve to equilibrium.
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
        text_bbs, bbox;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)

    # Continue from equilibrium with warm-start.
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
        text_bbs, bbox;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = false)

    # Own-anchor invariant: each rendered bbox does NOT contain its anchor.
    for i in 1:n
        rendered = Rect2f(text_bbs[i].origin[1] + offsets[i][1],
                          text_bbs[i].origin[2] + offsets[i][2],
                          text_bbs[i].widths[1],
                          text_bbs[i].widths[2])
        contains_x = textpositions[i][1] > rendered.origin[1] &&
                     textpositions[i][1] < rendered.origin[1] + rendered.widths[1]
        contains_y = textpositions[i][2] > rendered.origin[2] &&
                     textpositions[i][2] < rendered.origin[2] + rendered.widths[2]
        @test !(contains_x && contains_y)
    end
end

@testset "dispatch — reset=false with mismatched offsets length errors" begin
    alg = TextRepelAlgorithm()
    textpositions = [Point2f(0, 0), Point2f(100, 0)]
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    # offsets length is 1 but textpositions length is 2.
    offsets = [Vec2f(0, 0)]
    @test_throws DimensionMismatch Makie.calculate_best_offsets!(
        alg, offsets, textpositions, fill(Point2f(NaN, NaN), 2),
        text_bbs, Rect2f(0, 0, 500, 500);
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = false)
end

@testset "dispatch — n=1" begin
    textpositions = [Point2f(50, 50)]
    textpositions_offset = [Point2f(NaN, NaN)]
    text_bbs = [Rect2f(40, 45, 20, 10)]
    bbox     = Rect2f(0, 0, 100, 100)
    offsets  = [Vec2f(0, 0)]

    Makie.calculate_best_offsets!(TextRepelAlgorithm(),
                                  offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    @test all(isfinite, offsets[1])
end

@testset "dispatch — coincident anchors" begin
    n = 2
    textpositions = [Point2f(50, 50), Point2f(50, 50)]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(40, 45, 20, 10) for _ in 1:n]
    bbox     = Rect2f(0, 0, 200, 200)
    offsets  = [Vec2f(0, 0) for _ in 1:n]

    # Solver should not throw or produce NaN offsets for coincident anchors.
    Makie.calculate_best_offsets!(TextRepelAlgorithm(),
                                  offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    @test all(isfinite, offsets[1])
    @test all(isfinite, offsets[2])
    # Golden-angle init_state perturbation fans them out → different offsets.
    @test offsets[1] != offsets[2]
end

@testset "dispatch — zero-width label" begin
    textpositions = [Point2f(50, 50), Point2f(100, 50)]
    textpositions_offset = fill(Point2f(NaN, NaN), 2)
    text_bbs = [Rect2f(50, 50, 0, 0), Rect2f(90, 45, 20, 10)]
    bbox     = Rect2f(0, 0, 200, 200)
    offsets  = [Vec2f(0, 0) for _ in 1:2]

    # Solver should not divide by zero or produce NaN offsets.
    Makie.calculate_best_offsets!(TextRepelAlgorithm(),
                                  offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)
    @test all(isfinite, offsets[1])
    @test all(isfinite, offsets[2])
end

@testset "dispatch — NaN in textpositions errors clearly" begin
    textpositions = [Point2f(NaN, 50), Point2f(100, 50)]
    textpositions_offset = fill(Point2f(NaN, NaN), 2)
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 200, 200)
    offsets  = [Vec2f(0, 0) for _ in 1:2]

    @test_throws ArgumentError Makie.calculate_best_offsets!(
        TextRepelAlgorithm(),
        offsets, textpositions, textpositions_offset,
        text_bbs, bbox;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)
end

@testset "dispatch — obstacles avoidance" begin
    n = 1
    # Place a single label with initial bbox overlapping an obstacle.
    # The solver should displace it to avoid the obstacle.
    textpositions = [Point2f(50, 50)]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    # Bbox: 40-60 in x, 45-55 in y. This overlaps with the obstacle below.
    text_bbs = [Rect2f(40, 45, 20, 10)]
    bbox     = Rect2f(0, 0, 200, 100)

    # Obstacle at 35-55 in x, 48-68 in y — overlaps with the label's initial bbox.
    obstacle = Rect2f(35, 48, 20, 20)
    alg = TextRepelAlgorithm(obstacles = [obstacle], max_iter = 1000)
    offsets = [Vec2f(0, 0) for _ in 1:n]

    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    # After solve, the rendered bbox should not overlap the obstacle.
    rendered = Rect2f(text_bbs[1].origin[1] + offsets[1][1],
                      text_bbs[1].origin[2] + offsets[1][2],
                      text_bbs[1].widths[1],
                      text_bbs[1].widths[2])
    no_overlap = (rendered.origin[1] + rendered.widths[1] <= obstacle.origin[1] ||
                  obstacle.origin[1] + obstacle.widths[1] <= rendered.origin[1] ||
                  rendered.origin[2] + rendered.widths[2] <= obstacle.origin[2] ||
                  obstacle.origin[2] + obstacle.widths[2] <= rendered.origin[2])
    @test no_overlap
end

@testset "dispatch — maxiter is accepted but inert under ProjectionSolver" begin
    # maxiter has no effect under ProjectionSolver (no force-iteration cap);
    # it is honored only by ForceSolver. We assert only that passing it is
    # accepted and the solve succeeds. Both call shapes (an explicit Int and
    # Makie.automatic) are exercised to keep the dispatch contract covered.
    n = 2
    textpositions = [Point2f(0, 0), Point2f(100, 0)]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)

    alg = TextRepelAlgorithm(max_iter = 1000)
    offsets = [Vec2f(0, 0) for _ in 1:n]

    # Explicit maxiter — accepted, runs without error, finite offsets.
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = 5,
                                  labelspace = :relative_pixel,
                                  reset = true)
    @test all(o -> all(isfinite, o), offsets)
    @test solve_stats(alg).iter >= 0

    # Makie.automatic — same contract.
    fill!(offsets, Vec2f(0, 0))
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)
    @test all(o -> all(isfinite, o), offsets)
    @test solve_stats(alg).iter >= 0
end

@testset "dispatch — labelspace is a no-op (does not reach solver)" begin
    n = 2
    textpositions = [Point2f(0, 0), Point2f(100, 0)]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)

    alg = TextRepelAlgorithm()

    offsets_rel = [Vec2f(0, 0) for _ in 1:n]
    Makie.calculate_best_offsets!(alg, offsets_rel, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    offsets_data = [Vec2f(0, 0) for _ in 1:n]
    Makie.calculate_best_offsets!(alg, offsets_data, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :data,
                                  reset = true)

    # labelspace doesn't reach the solver — same input → same output.
    @test offsets_rel == offsets_data
end

@testset "dispatch — StaticArrays broadcast canary" begin
    # The dispatch uses per-element T(x, y) construction rather than
    # broadcast assignment to avoid StaticArrays broadcasting into each
    # element. This test asserts the path runs cleanly on Vector{Vec2f}.
    n = 3
    offsets = Vector{Vec2f}([Vec2f(0, 0) for _ in 1:n])
    textpositions = [Point2f(i * 50, 50) for i in 1:n]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]

    @test_nowarn Makie.calculate_best_offsets!(TextRepelAlgorithm(),
                                               offsets, textpositions, textpositions_offset,
                                               text_bbs, Rect2f(0, 0, 500, 500);
                                               maxiter = Makie.automatic,
                                               labelspace = :relative_pixel,
                                               reset = true)
end

@testset "dispatch — solve_stats populated after real solve" begin
    n = 3
    textpositions = [Point2f(i * 50, 50) for i in 1:n]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(p[1] - 10, p[2] - 5, 20, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)
    offsets  = [Vec2f(0, 0) for _ in 1:n]

    alg = TextRepelAlgorithm()
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    s = solve_stats(alg)
    @test s.iter >= 0
    @test s.overlaps == 0
    @test s.residual >= 0f0
end

@testset "stress smoke — n=100 random scatter" begin
    using Random
    rng = Random.MersenneTwister(0)
    n = 100
    textpositions = [Point2f(500 * rand(rng), 500 * rand(rng)) for _ in 1:n]
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(p[1] - 15, p[2] - 5, 30, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)
    offsets  = [Vec2f(0, 0) for _ in 1:n]

    alg = TextRepelAlgorithm(max_iter = 500)
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    # All offsets finite — no NaN/Inf leaked through the solver.
    @test all(o -> all(isfinite, o), offsets)
    # Solver made progress.
    @test solve_stats(alg).iter > 0
    # Rendered bboxes stay within a sanity envelope around the viewport
    # (allow some slack — bounds clamping may not be exact at extremes).
    for i in 1:n
        rendered_center = textpositions[i] .+ offsets[i]
        @test rendered_center[1] > -100 && rendered_center[1] < 600
        @test rendered_center[2] > -100 && rendered_center[2] < 600
    end
end

@testset "stress smoke — pathological co-located cluster (n=30)" begin
    # All 30 anchors at the same position. Solver should fan them out via
    # init_offsets's golden-angle spiral, not produce NaN/coincident bboxes.
    n = 30
    textpositions = fill(Point2f(250, 250), n)
    textpositions_offset = fill(Point2f(NaN, NaN), n)
    text_bbs = [Rect2f(p[1] - 25, p[2] - 5, 50, 10) for p in textpositions]
    bbox     = Rect2f(0, 0, 500, 500)
    offsets  = [Vec2f(0, 0) for _ in 1:n]

    alg = TextRepelAlgorithm(max_iter = 500)
    Makie.calculate_best_offsets!(alg, offsets, textpositions, textpositions_offset,
                                  text_bbs, bbox;
                                  maxiter = Makie.automatic,
                                  labelspace = :relative_pixel,
                                  reset = true)

    # No NaN/Inf.
    @test all(o -> all(isfinite, o), offsets)
    # Labels spread out via init_offsets's golden-angle spiral. Each of
    # the 30 indices produces a distinct direction, so under any
    # non-collapsed solve the offsets remain distinct. Threshold > n/2
    # catches partial-collapse regressions where a chunk of labels
    # converge to the same point.
    @test length(Set(offsets)) > n ÷ 2
end

@testset "Stability canary — Makie.calculate_best_offsets! dispatches" begin
    # Confirms the hook symbol exists and dispatches on a TextRepelAlgorithm
    # with the documented kwargs. If a Makie upgrade renames or re-signatures
    # this hook, this test fails loudly in CI before users hit it at runtime.

    @test isdefined(Makie, :calculate_best_offsets!)
    # Check method signature exists with the right positional and keyword arguments.
    # If the signature changed, this will throw.
    Makie.calculate_best_offsets!(
        TextRepelAlgorithm(),
        Vec2f[], Point2f[], Point2f[], Rect2f[], Rect2f(0, 0, 1, 1);
        maxiter = Makie.automatic,
        labelspace = :relative_pixel,
        reset = true)
    @test true  # If no error, the signature is correct.
end

@testset "annotation fresh path produces crossing-free leaders (#12)" begin
    # Invariant: the fresh annotation path reaches solve_cluster, so its leaders are
    # crossing-free. (That repair is *responsible* — vs voronoi-init alone — is proven
    # deterministically at the solve_cluster level in Task 4's fresh-path test; this is
    # the annotation-surface end-to-end invariant.)
    anchors = [Point2f(0, 0), Point2f(40, 0), Point2f(0, 40), Point2f(40, 40)]
    n = length(anchors)
    bbox = Rect2f(-100, -100, 300, 300)
    # centered text bbs: origin = anchor - widths/2 → align_bias = 0
    w = Vec2f(20, 10)
    text_bbs = [Rect2f(a[1]-w[1]/2, a[2]-w[2]/2, w[1], w[2]) for a in anchors]
    tpos_off = fill(Point2f(NaN, NaN), n)        # all auto-placed
    offsets  = fill(Vec2f(0, 0), n)

    alg = TextRepelAlgorithm()
    Makie.calculate_best_offsets!(alg, offsets, anchors, tpos_off, text_bbs, bbox;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)

    # Build leader connectors at the solved positions and assert none cross.
    sizes = [Vec2f(bb.widths...) for bb in text_bbs]
    conns = [MakieTextRepel.connector_for(anchors[i], Vec2f(offsets[i]...), sizes[i],
                                          false, RepelParams(), 0.0) for i in 1:n]
    @test isempty(MakieTextRepel.find_crossings(conns))
end

@testset "annotation fresh path is deterministic (#12)" begin
    anchors = [Point2f(0,0), Point2f(10,0), Point2f(10,0), Point2f(0,10)]  # incl. coincident
    n = length(anchors)
    bbox = Rect2f(-100, -100, 300, 300)
    w = Vec2f(16, 8)
    text_bbs = [Rect2f(a[1]-w[1]/2, a[2]-w[2]/2, w[1], w[2]) for a in anchors]
    tpos_off = fill(Point2f(NaN, NaN), n)

    run() = (o = fill(Vec2f(0,0), n);
             Makie.calculate_best_offsets!(TextRepelAlgorithm(), o, anchors, tpos_off,
                 text_bbs, bbox; maxiter = Makie.automatic, labelspace = :relative_pixel,
                 reset = true); o)
    o1 = run(); o2 = run()
    @test o1 == o2                       # seeded voronoi → identical across runs
end

@testset "annotation fresh path separates coincident anchors (#12, spec item 5)" begin
    # Coincident anchors (2,3) must not collapse onto the same rendered position.
    anchors = [Point2f(0,0), Point2f(10,0), Point2f(10,0), Point2f(0,10)]
    n = length(anchors)
    bbox = Rect2f(-100, -100, 300, 300)
    w = Vec2f(16, 8)
    text_bbs = [Rect2f(a[1]-w[1]/2, a[2]-w[2]/2, w[1], w[2]) for a in anchors]
    tpos_off = fill(Point2f(NaN, NaN), n)
    o = fill(Vec2f(0,0), n)
    Makie.calculate_best_offsets!(TextRepelAlgorithm(), o, anchors, tpos_off, text_bbs, bbox;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)
    @test anchors[2] .+ Vec2f(o[2]...) != anchors[3] .+ Vec2f(o[3]...)
end

@testset "annotation non-centered text places sanely (#12, dismissed Risk 3)" begin
    # Left/bottom-aligned text: bbox origin AT the anchor → align_bias = widths/2 ≠ 0.
    anchors = [Point2f(0, 0), Point2f(30, 0), Point2f(0, 30)]
    n = length(anchors)
    bbox = Rect2f(-100, -100, 300, 300)
    w = Vec2f(20, 10)
    text_bbs = [Rect2f(a[1], a[2], w[1], w[2]) for a in anchors]   # origin = anchor
    tpos_off = fill(Point2f(NaN, NaN), n)
    offsets  = fill(Vec2f(0, 0), n)

    alg = TextRepelAlgorithm()
    Makie.calculate_best_offsets!(alg, offsets, anchors, tpos_off, text_bbs, bbox;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)

    # All offsets finite, and each label is pushed off its own anchor (not collapsed
    # onto it) — own-anchor repulsion holds regardless of alignment.
    for i in 1:n
        @test all(isfinite, offsets[i])
        rendered_center = anchors[i] .+ Vec2f(offsets[i]...) .+ w ./ 2
        @test rendered_center != anchors[i]
    end
end

@testset "annotation all-pinned bypass preserved (#12)" begin
    anchors = [Point2f(0,0), Point2f(20,0)]
    n = length(anchors)
    bbox = Rect2f(-50,-50,100,100)
    w = Vec2f(10,6)
    text_bbs = [Rect2f(a[1]-w[1]/2, a[2]-w[2]/2, w[1], w[2]) for a in anchors]
    # all finite textpositions_offset → all pinned. Pinned render offset = tpo - anchor.
    tpos_off = [Point2f(3, 3), Point2f(23, -4)]
    offsets  = fill(Vec2f(0,0), n)
    alg = TextRepelAlgorithm()
    Makie.calculate_best_offsets!(alg, offsets, anchors, tpos_off, text_bbs, bbox;
        maxiter = Makie.automatic, labelspace = :relative_pixel, reset = true)
    @test offsets[1] == Vec2f(3, 3)
    @test offsets[2] == Vec2f(3, -4)
    @test solve_stats(alg).iter == 0     # bypass: solver never ran
end
