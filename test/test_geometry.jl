using MakieTextRepel: box_at, overlap_push, point_push, clip_to_box_edge, clamp_box_offset
using GeometryBasics

@testset "geometry" begin
    # box_at centers a size on anchor+offset
    b = box_at(Point2f(10, 10), Vec2f(2, 0), Vec2f(4, 6))
    @test b.origin ≈ Point2f(10, 7)        # (12,10) - (2,3)
    @test b.widths ≈ Vec2f(4, 6)

    # overlap_push: non-overlapping boxes return zero
    a = box_at(Point2f(0, 0), Vec2f(0, 0), Vec2f(2, 2))
    far = box_at(Point2f(10, 0), Vec2f(0, 0), Vec2f(2, 2))
    @test overlap_push(a, far) == Vec2f(0, 0)

    # overlap_push: overlapping boxes push a away from b on the overlapping axes
    near = box_at(Point2f(1, 0), Vec2f(0, 0), Vec2f(2, 2))  # overlaps a by 1 in x
    push = overlap_push(a, near)
    @test push[1] < 0          # a is left of near, pushed further left
    @test abs(push[1]) ≈ 1.0   # overlap extent on x
    @test push[2] == 0         # boxes share y (aligned axis) → no y push

    # point_push: point outside box returns zero
    box = box_at(Point2f(0, 0), Vec2f(0, 0), Vec2f(4, 4))
    @test point_push(box, Point2f(10, 10), 0f0) == Vec2f(0, 0)

    # point_push: point inside box pushes box away from point
    pp = point_push(box, Point2f(1, 0), 0f0)
    @test pp[1] < 0            # point right-of-center → box pushed left

    # clip_to_box_edge: point on the box boundary toward the target
    edge = clip_to_box_edge(box, Point2f(100, 0))   # target far to the right
    @test edge ≈ Point2f(2, 0)                       # right edge at x=+2
end

@testset "clamp_box_offset" begin
    bounds = Rect2f(0, 0, 100, 100)

    # fully inside → zero shift
    @test clamp_box_offset(Rect2f(10, 10, 20, 20), bounds) == Vec2f(0, 0)
    # over the right edge → pushed left by the overshoot
    @test clamp_box_offset(Rect2f(90, 10, 20, 20), bounds) ≈ Vec2f(-10, 0)
    # over the bottom edge → pushed up
    @test clamp_box_offset(Rect2f(10, -5, 20, 20), bounds) ≈ Vec2f(0, 5)
    # over left and top → pushed right and down
    @test clamp_box_offset(Rect2f(-5, 90, 20, 20), bounds) ≈ Vec2f(5, -10)
    # wider than bounds on x → pinned to the lower (left) edge: origin.x → 0
    @test clamp_box_offset(Rect2f(20, 10, 200, 20), bounds) ≈ Vec2f(-20, 0)
end

@testset "clip_to_box_edge: inside-box and boundary" begin
    box = box_at(Point2f(0, 0), Vec2f(0, 0), Vec2f(4, 4))   # box: x ∈ [-2, 2], y ∈ [-2, 2]

    # strictly inside on both axes → nothing
    @test clip_to_box_edge(box, Point2f(0.5, 0.5)) === nothing

    # exact center → nothing (subsumed by strict-inside)
    @test clip_to_box_edge(box, Point2f(0, 0)) === nothing

    # target on a face (|d_x| = hw, |d_y| < hh) → valid face endpoint at t = 1
    edge = clip_to_box_edge(box, Point2f(2, 0))
    @test edge !== nothing
    @test edge ≈ Point2f(2, 0)

    # target on a corner (|d_x| = hw and |d_y| = hh) → valid corner endpoint
    edge_c = clip_to_box_edge(box, Point2f(2, 2))
    @test edge_c !== nothing
    @test edge_c ≈ Point2f(2, 2)

    # target outside on x only → near face on x, clipped (regression of existing case)
    @test clip_to_box_edge(box, Point2f(100, 0)) ≈ Point2f(2, 0)

    # target outside on both axes diagonally → corner of the box on the limiting axis
    # For (4, 4): t = min(2/4, 2/4) = 0.5, edge = (2, 2)
    @test clip_to_box_edge(box, Point2f(4, 4)) ≈ Point2f(2, 2)
end
