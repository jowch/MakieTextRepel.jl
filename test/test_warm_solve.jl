using MakieTextRepel
using MakieTextRepel: warm_solve, solve_cluster, ProjectionSolver, RepelParams
using GeometryBasics
using Test

# A small, well-separated fixture (sparse → fresh solve leaves small offsets).
const WS_BOUNDS  = Rect2f(0, 0, 400, 400)
const WS_ANCHORS = Point2f[(60, 60), (200, 80), (120, 260), (320, 200), (260, 330)]
const WS_SIZES   = Vec2f[(40, 16) for _ in 1:5]

@testset "warm_solve" begin
    @testset "fresh solve: shape + invariants" begin
        r = warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS)
        # Documented return shape: the full 4-tuple, forwarded from solve_cluster.
        @test propertynames(r) == (:offsets, :dropped, :iter, :residual)
        @test r.offsets isa Vector{Vec2f}
        @test r.dropped isa BitVector
        @test length(r.offsets) == length(WS_ANCHORS)
        @test length(r.dropped) == length(WS_ANCHORS)
        @test r.iter isa Int
        @test r.residual isa Float32
        @test all(o -> all(isfinite, o), r.offsets)
    end

    @testset "deterministic: same inputs ⇒ identical output" begin
        a = warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS)
        b = warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS)
        @test a.offsets == b.offsets
        @test a.dropped == b.dropped
    end

    @testset "equivalent to a direct solve_cluster call (seam tie)" begin
        # warm_solve must be a transparent forward over the internal primitive with
        # the same config. Build the identical RepelParams the wrapper builds.
        p = RepelParams(; only_move = :both, box_padding = 4.0,
                          point_padding = 5.0, min_segment_length = 2.0)
        direct = solve_cluster(ProjectionSolver(p), WS_ANCHORS, WS_SIZES, WS_BOUNDS)
        r = warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS)
        @test r.offsets == direct.offsets
        @test r.dropped == direct.dropped
    end

    @testset "config kwargs forward (point_padding default = 5.0)" begin
        # The wrapper default point_padding is the user-surface 5.0, not RepelParams' 0.0.
        p5 = RepelParams(; only_move = :both, box_padding = 4.0,
                           point_padding = 5.0, min_segment_length = 2.0)
        p0 = RepelParams(; only_move = :both, box_padding = 4.0,
                           point_padding = 0.0, min_segment_length = 2.0)
        d_default = solve_cluster(ProjectionSolver(p5), WS_ANCHORS, WS_SIZES, WS_BOUNDS)
        d_zero    = solve_cluster(ProjectionSolver(p0), WS_ANCHORS, WS_SIZES, WS_BOUNDS)
        @test warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS).offsets == d_default.offsets
        @test warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS;
                         point_padding = 0.0).offsets == d_zero.offsets
        # Guard against "point_padding silently ignored": the two configs must
        # actually produce different placements on this fixture.
        @test d_default.offsets != d_zero.offsets
    end

    @testset "warm-start path forwards init_state" begin
        # Warm-start from the fresh result: a second relax pass on an already-legal
        # layout reproduces solve_cluster's warm path exactly.
        fresh = warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS)
        p = RepelParams(; only_move = :both, box_padding = 4.0,
                          point_padding = 5.0, min_segment_length = 2.0)
        direct = solve_cluster(ProjectionSolver(p), WS_ANCHORS, WS_SIZES, WS_BOUNDS;
                               init_state = fresh.offsets)
        warm = warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS; init_state = fresh.offsets)
        @test warm.offsets == direct.offsets
        @test warm.dropped == direct.dropped
    end

    @testset "pin + obstacles forward" begin
        pin = falses(length(WS_ANCHORS)); pin[1] = true
        pinned = Vec2f[Vec2f(0, 0) for _ in 1:length(WS_ANCHORS)]
        pinned[1] = Vec2f(30, 30)
        # This obstacle is NOT inert: it overlaps non-pinned label 2's default slot
        # and forces it to move (verified: [0,13] → [-54,13] vs the no-obstacle
        # solve below). A degenerate obstacle would let the equivalence assertion
        # pass even if warm_solve dropped the kwarg — so the guard below makes the
        # obstacle's effect explicit, mirroring the point_padding self-verification.
        obs = Rect2f[Rect2f(170, 50, 110, 90)]
        p = RepelParams(; only_move = :both, box_padding = 4.0,
                          point_padding = 5.0, min_segment_length = 2.0)
        direct = solve_cluster(ProjectionSolver(p), WS_ANCHORS, WS_SIZES, WS_BOUNDS;
                               pin_mask = pin, pinned_offsets = pinned, obstacles = obs)
        warm = warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS;
                          pin_mask = pin, pinned_offsets = pinned, obstacles = obs)
        noobs = warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS;
                           pin_mask = pin, pinned_offsets = pinned)
        @test warm.offsets == direct.offsets         # forwards pin + obstacles to the seam
        @test warm.offsets[1] == Vec2f(30, 30)       # pinned label held at its fixed offset
        @test warm.offsets != noobs.offsets          # the obstacle actually changed placement
    end

    @testset "validates sizes length at the public boundary" begin
        @test_throws DimensionMismatch warm_solve(WS_ANCHORS, WS_SIZES[1:end-1], WS_BOUNDS)
    end
end
