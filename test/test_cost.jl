using MakieTextRepel
using MakieTextRepel: label_cost
using GeometryBasics
using Test

@testset "label_cost" begin
    bounds = Rect2f(0, 0, 200, 200)
    bp = 0.0   # zero padding so geometry is easy to reason about

    # Two 20×20 labels. anchors at (50,50) and (50,50); offsets put centers
    # 10px apart in x → boxes (half-width 10 each) penetrate 10px on x, fully overlap y.
    anchors = [Point2f(50, 50), Point2f(50, 50)]
    sizes   = [Vec2f(20, 20), Vec2f(20, 20)]
    offs_ov = [Vec2f(0, 0), Vec2f(10, 0)]
    q = label_cost(anchors, sizes; offsets = offs_ov, bounds = bounds,
                   box_padding = bp, min_segment_length = 2.0)
    @test q.overlaps == 1                       # 10px > 0.5 both axes
    @test q.mean_leader ≈ Float32((0 + 10) / 2) # ‖(0,0)‖=0, ‖(10,0)‖=10

    # Separate them 40px in x → centers 40 apart, half-widths sum 20 → no overlap.
    offs_sep = [Vec2f(0, 0), Vec2f(40, 0)]
    q2 = label_cost(anchors, sizes; offsets = offs_sep, bounds = bounds,
                    box_padding = bp, min_segment_length = 2.0)
    @test q2.overlaps == 0

    # Sub-0.5px touch is NOT counted as an overlap (the Q threshold split).
    offs_near = [Vec2f(0, 0), Vec2f(19.7, 0)]  # penetration 20-19.7 = 0.3 < 0.5 → not counted
    @test label_cost(anchors, sizes; offsets = offs_near, bounds = bounds,
                     box_padding = bp, min_segment_length = 2.0).overlaps == 0

    # Dropped labels are excluded from overlaps and mean_leader.
    dropped = BitVector([false, true])
    q3 = label_cost(anchors, sizes; offsets = offs_ov, bounds = bounds, dropped = dropped,
                    box_padding = bp, min_segment_length = 2.0)
    @test q3.overlaps == 0                       # label 2 dropped → pair excluded
    @test q3.mean_leader ≈ 0f0                   # only label 1 active, ‖(0,0)‖ = 0

    # crossings: cross-check against a hand-built crossing fixture (two leaders that cross).
    a = [Point2f(0, 0), Point2f(10, 0)]
    s = [Vec2f(4, 4), Vec2f(4, 4)]
    o = [Vec2f(10, 4), Vec2f(-10, 4)]            # label1 goes up-right, label2 up-left → cross
    qc = label_cost(a, s; offsets = o, bounds = Rect2f(-50, -50, 100, 100),
                    box_padding = bp, min_segment_length = 0.5)
    @test qc.crossings == 1
end

@testset "label_cost point_overlaps" begin
    # Two anchors 30px apart on x. Label 1 (wide) at offset 0 covers anchor 2.
    anchors = [Point2f(100, 100), Point2f(130, 100)]
    sizes   = [Vec2f(80, 20), Vec2f(10, 10)]
    offs    = [Vec2f(0, 0), Vec2f(0, 40)]      # label 1 centered on its anchor → covers anchor 2
    bounds  = Rect2f(0, 0, 400, 400)
    q = label_cost(anchors, sizes; offsets = offs, bounds = bounds,
                   box_padding = 0.0, point_padding = 2.0, min_segment_length = 4.0)
    @test q.point_overlaps == 1                # anchor 2 sits under label 1's box
    # move label 1 far up so it no longer covers anchor 2
    offs2 = [Vec2f(0, 80), Vec2f(0, 40)]
    q2 = label_cost(anchors, sizes; offsets = offs2, bounds = bounds,
                    box_padding = 0.0, point_padding = 2.0, min_segment_length = 4.0)
    @test q2.point_overlaps == 0
end
