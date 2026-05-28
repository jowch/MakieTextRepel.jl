using MakieTextRepel
using MakieTextRepel: segments_cross, connector_for, Connector, RepelParams
using GeometryBasics
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
