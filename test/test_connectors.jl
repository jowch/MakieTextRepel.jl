using MakieTextRepel: build_connectors
using GeometryBasics
using LinearAlgebra

@testset "build_connectors" begin
    anchors = [Point2f(0, 0), Point2f(50, 0)]
    sizes = [Vec2f(20, 10), Vec2f(20, 10)]
    dropped = falses(2)

    # label 1 moved far (offset 30 in x), label 2 not moved
    offsets = [Vec2f(30, 0), Vec2f(0, 0)]
    segs = build_connectors(anchors, offsets, sizes, dropped, 5.0, 0.0)
    @test length(segs) == 2                       # one segment = 2 endpoints
    @test segs[1] == Point2f(0, 0)                # starts at anchor 1
    @test segs[2][1] < 30 && segs[2][1] > 0       # ends on the box's near edge

    # min_segment_length suppresses the short one
    segs2 = build_connectors(anchors, [Vec2f(2, 0), Vec2f(0, 0)], sizes, dropped, 5.0, 0.0)
    @test isempty(segs2)

    # dropped labels produce no connector
    segs3 = build_connectors(anchors, offsets, sizes, BitVector([true, false]), 5.0, 0.0)
    @test isempty(segs3)
end
