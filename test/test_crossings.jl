using MakieTextRepel
using MakieTextRepel: segments_cross, connector_for, Connector, RepelParams
using GeometryBasics
using LinearAlgebra: norm
using Test

@testset "segments_cross" begin
    # Two segments forming an X
    @test segments_cross(Point2f(0, 0), Point2f(2, 2), Point2f(0, 2), Point2f(2, 0)) == true

    # Parallel non-coincident
    @test segments_cross(Point2f(0, 0), Point2f(2, 0), Point2f(0, 1), Point2f(2, 1)) == false

    # Disjoint, non-parallel
    @test segments_cross(Point2f(0, 0), Point2f(1, 0), Point2f(2, 1), Point2f(3, 2)) == false

    # Endpoint touch — not a crossing
    @test segments_cross(Point2f(0, 0), Point2f(1, 1), Point2f(1, 1), Point2f(2, 0)) == false

    # T-junction: one endpoint exactly on the other segment — not a crossing
    @test segments_cross(Point2f(0, 0), Point2f(2, 0), Point2f(1, 0), Point2f(1, 2)) == false

    # Collinear overlap — not a crossing
    @test segments_cross(Point2f(0, 0), Point2f(3, 0), Point2f(1, 0), Point2f(2, 0)) == false
end

@testset "connector_for" begin
    params = RepelParams(box_padding = 4.0, point_padding = 2.0)
    # Label offset to the right of anchor with non-zero length leader → drawn.
    c = connector_for(Point2f(0, 0), Vec2f(20, 0), Vec2f(10, 6), false, params, 2.0)
    @test c.drawn == true

    # Dropped label → not drawn.
    c2 = connector_for(Point2f(0, 0), Vec2f(20, 0), Vec2f(10, 6), true, params, 2.0)
    @test c2.drawn == false

    # Anchor inside padded box (offset 0) → not drawn.
    c3 = connector_for(Point2f(0, 0), Vec2f(0, 0), Vec2f(10, 6), false, params, 2.0)
    @test c3.drawn == false

    # Visible length below min_segment_length → not drawn.
    c4 = connector_for(Point2f(0, 0), Vec2f(11, 0), Vec2f(10, 6), false, params, 100.0)
    @test c4.drawn == false
end

using MakieTextRepel: find_crossings

@testset "find_crossings" begin
    # Two connectors that cross.
    c1 = Connector(Point2f(2, 0), Point2f(0, 2), true)
    c2 = Connector(Point2f(2, 2), Point2f(0, 0), true)
    @test find_crossings([c1, c2]) == [(1, 2)]

    # Same but second one undrawn → no crossing.
    c2_off = Connector(c2.label_end, c2.anchor_end, false)
    @test find_crossings([c1, c2_off]) == Tuple{Int,Int}[]

    # Three connectors, two pairwise crossings.
    c3 = Connector(Point2f(0, 0), Point2f(2, 2), true)
    c4 = Connector(Point2f(2, 0), Point2f(0, 2), true)
    c5 = Connector(Point2f(0, 1), Point2f(2, 1), true)
    crossings = find_crossings([c3, c4, c5])
    @test (1, 2) in crossings
    @test issorted(crossings)  # lex-ordered

    # No crossings on parallel lines.
    c6 = Connector(Point2f(0, 0), Point2f(2, 0), true)
    c7 = Connector(Point2f(0, 1), Point2f(2, 1), true)
    @test find_crossings([c6, c7]) == Tuple{Int,Int}[]
end

using MakieTextRepel: swap_positions!

@testset "swap_positions!" begin
    anchors = [Point2f(0, 0), Point2f(10, 0)]
    offsets = [Vec2f(5, 5), Vec2f(-5, 5)]
    # absolute positions: (5, 5) and (5, 5)
    # Yes those are coincident — pick more interesting ones:
    offsets = [Vec2f(2, 3), Vec2f(-1, 4)]
    # absolute positions: (2, 3) and (9, 4)
    swap_positions!(offsets, anchors, 1, 2)
    # After: label 1 at (9, 4), label 2 at (2, 3)
    # offsets[1] = (9, 4) - (0, 0) = (9, 4)
    # offsets[2] = (2, 3) - (10, 0) = (-8, 3)
    @test offsets[1] ≈ Vec2f(9, 4)
    @test offsets[2] ≈ Vec2f(-8, 3)
end

using MakieTextRepel: repair_crossings!

@testset "repair_crossings!" begin
    # Construct a 2-label crossing: anchors on x-axis, labels swapped across.
    anchors = [Point2f(0, 0), Point2f(10, 0)]
    offsets = [Vec2f(12, 4), Vec2f(-12, 4)]
    # absolute positions: label_1 at (12, 4) — leader (0,0)→(12,4); label_2 at (-2, 4) — leader (10,0)→(-2,4). Cross.
    sizes = [Vec2f(4, 2), Vec2f(4, 2)]
    dropped = BitVector([false, false])
    params = RepelParams(box_padding = 1.0, point_padding = 0.5)

    iters = repair_crossings!(offsets, anchors, sizes, dropped, params; min_len = 0.5)
    @test iters ≤ 5  # should converge in 1 iteration
    # Verify no crossings remain after repair.
    connectors = [connector_for(anchors[i], offsets[i], sizes[i], dropped[i], params, 0.5)
                  for i in eachindex(anchors)]
    @test isempty(find_crossings(connectors))

    # Sum of center-distance has decreased.
    @test norm(offsets[1]) + norm(offsets[2]) ≤ norm(Vec2f(12, 4)) + norm(Vec2f(-12, 4))

    # Dropped label: its slot is skipped, others repair around it.
    anchors2 = [Point2f(0, 0), Point2f(10, 0), Point2f(5, 5)]
    offsets2 = [Vec2f(12, 4), Vec2f(-12, 4), Vec2f(0, 0)]
    sizes2 = [Vec2f(4, 2), Vec2f(4, 2), Vec2f(4, 2)]
    dropped2 = BitVector([false, false, true])
    repair_crossings!(offsets2, anchors2, sizes2, dropped2, params; min_len = 0.5)
    # Label 3 should never appear in any crossing pair → its offset is unchanged.
    @test offsets2[3] == Vec2f(0, 0)

    # max_iter cap: pathological alternating input doesn't hang.
    # (Hard to construct a true cycle in floating point, so just assert termination.)
    offsets3 = [Vec2f(12, 4), Vec2f(-12, 4)]
    iters3 = repair_crossings!(offsets3, anchors, sizes, dropped, params; max_iter = 3, min_len = 0.5)
    @test iters3 ≤ 3

    # Cap-out signal: drive a crossing input with `max_iter = 0` so the loop
    # body never runs, the final rescan finds the un-repaired crossing, and the
    # function emits the @warn that the recipe (and any other caller) needs to
    # see in order to know the no-crossing property failed for this layout.
    offsets4 = [Vec2f(12, 4), Vec2f(-12, 4)]
    @test_logs (:warn, r"max_iter=0") (@test repair_crossings!(offsets4, anchors, sizes, dropped, params;
                                                               max_iter = 0, min_len = 0.5) == 0)
    # Residual crossing must still be present in the offsets we returned.
    conn4 = [connector_for(anchors[i], offsets4[i], sizes[i], dropped[i], params, 0.5)
             for i in eachindex(anchors)]
    @test !isempty(find_crossings(conn4))

    # Conversely, a no-crossings input with `max_iter = 0` must NOT warn and
    # must return 0 (the loop's "converged at iter-1=0" path).
    offsets5 = [Vec2f(5, 5), Vec2f(5, 5)]   # parallel leaders, no crossing
    @test_logs repair_crossings!(offsets5, anchors, sizes, dropped, params;
                                 max_iter = 0, min_len = 0.5)
end
