using MakieTextRepel: build_connectors
using GeometryBasics
using LinearAlgebra

@testset "build_connectors: basic" begin
    anchors = [Point2f(0, 0), Point2f(50, 0)]
    sizes = [Vec2f(20, 10), Vec2f(20, 10)]
    dropped = falses(2)

    # label 1 moved far (offset 30 in x), label 2 not moved
    offsets = [Vec2f(30, 0), Vec2f(0, 0)]
    segs = build_connectors(anchors, offsets, sizes, dropped, 5.0, 0.0)
    # label 2 has anchor at center → suppressed by strict-inside
    # label 1: anchor at (0,0); box center at (30,0); box extends x∈[20,40], so anchor
    # outside on x. edge = (20, 0). Visible length = 20 > 5 → emitted.
    @test length(segs) == 2
    @test segs[1] == Point2f(0, 0)
    @test segs[2][1] ≈ 20.0f0

    # dropped labels produce no connector
    segs_drop = build_connectors(anchors, offsets, sizes, BitVector([true, false]), 5.0, 0.0)
    @test isempty(segs_drop)
end

@testset "build_connectors: anchor trim by point_padding" begin
    # label 1 offset 30 in x; anchor at (0,0), label center at (30,0). With
    # point_padding = 4, segment start should be 4 px in along +x: (4, 0).
    anchors = [Point2f(0, 0)]
    sizes = [Vec2f(20, 10)]
    offsets = [Vec2f(30, 0)]
    dropped = falses(1)
    segs = build_connectors(anchors, offsets, sizes, dropped, 0.0, 0.0;
                            point_padding = 4.0)
    @test length(segs) == 2
    @test segs[1] ≈ Point2f(4, 0)        # start trimmed by 4 px
    @test segs[2] ≈ Point2f(20, 0)       # end at box face (unchanged)
end

@testset "build_connectors: anchor inside padded box is suppressed (locking)" begin
    # This locks behavior already established by Task 1's strict-inside check.
    # Task 2 must preserve it.
    # Box at offset (5, 0), size (20, 10) → box x ∈ [-5, 15]. Anchor at (0, 0)
    # is strictly inside → no segment.
    anchors = [Point2f(0, 0)]
    sizes = [Vec2f(20, 10)]
    offsets = [Vec2f(5, 0)]
    dropped = falses(1)
    segs = build_connectors(anchors, offsets, sizes, dropped, 0.0, 0.0)
    @test isempty(segs)
end

@testset "build_connectors: fan-out across coincident-anchor labels" begin
    # Three labels at the same anchor with three distinct offsets. Each emits a
    # segment in a distinct direction (no uniform +x bias). We construct the
    # offsets directly here (we are testing build_connectors, not the solver).
    anchors = fill(Point2f(0, 0), 3)
    sizes = [Vec2f(20, 10), Vec2f(20, 10), Vec2f(20, 10)]
    offsets = [Vec2f(30, 0), Vec2f(-20, 25), Vec2f(0, -30)]
    dropped = falses(3)
    segs = build_connectors(anchors, offsets, sizes, dropped, 0.0, 0.0)
    @test length(segs) == 6   # three segments × two endpoints
    # Three distinct edge endpoints (segs[2], segs[4], segs[6]) — no bias.
    ends = (segs[2], segs[4], segs[6])
    @test ends[1] != ends[2] && ends[2] != ends[3] && ends[1] != ends[3]
    # Each segment direction is roughly toward its offset (sanity for no bias).
    @test segs[2][1] > 0                          # label 1 ends to the right
    @test segs[4][1] < 0 && segs[4][2] > 0        # label 2 ends upper-left
    @test segs[6][2] < 0                          # label 3 ends below
end

@testset "build_connectors: visible-length filter" begin
    # Anchor outside box but only by 1 px; with min_len = 2 the segment is
    # suppressed even though norm(offset) is large.
    # box at offset (11, 0), size (20, 10) → box x ∈ [1, 21]. Anchor at (0, 0).
    # edge = (1, 0). Visible length = 1.0 < min_len 2.0 → suppressed.
    anchors = [Point2f(0, 0)]
    sizes = [Vec2f(20, 10)]
    offsets = [Vec2f(11, 0)]
    dropped = falses(1)
    segs = build_connectors(anchors, offsets, sizes, dropped, 2.0, 0.0)
    @test isempty(segs)

    # Same setup but offset = (13, 0): box x ∈ [3, 23]. Anchor at (0, 0). Edge =
    # (3, 0). Visible length = 3.0 > 2.0 → emitted.
    segs2 = build_connectors(anchors, [Vec2f(13, 0)], sizes, dropped, 2.0, 0.0)
    @test length(segs2) == 2
    @test segs2[2] ≈ Point2f(3, 0)
end

@testset "build_connectors: diagonal offset terminates on the limiting face" begin
    # Square box, equal diagonal offset → t = min(hw/|dx|, hh/|dy|), both equal,
    # corner endpoint.
    anchors = [Point2f(0, 0)]
    sizes = [Vec2f(10, 10)]
    offsets = [Vec2f(20, 20)]   # box center at (20, 20), box [15..25] × [15..25]
    dropped = falses(1)
    segs = build_connectors(anchors, offsets, sizes, dropped, 0.0, 0.0)
    @test length(segs) == 2
    @test segs[1] == Point2f(0, 0)
    @test segs[2] ≈ Point2f(15, 15)   # near corner of the box
end

@testset "build_connectors: keyword default for point_padding is 0.0" begin
    # Passing no point_padding keyword should match passing point_padding = 0.0.
    anchors = [Point2f(0, 0)]
    sizes = [Vec2f(20, 10)]
    offsets = [Vec2f(30, 0)]
    dropped = falses(1)
    a = build_connectors(anchors, offsets, sizes, dropped, 0.0, 0.0)
    b = build_connectors(anchors, offsets, sizes, dropped, 0.0, 0.0;
                         point_padding = 0.0)
    @test a == b
end
