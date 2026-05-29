using MakieTextRepel
using MakieTextRepel: side_select, slot_offset, box_at, overlap_push, IMHOF_ORDER, RepelParams
using GeometryBasics
using Test

psz(sizes, bp) = [s .+ Vec2f(2bp, 2bp) for s in sizes]
anyov(anchors, sel, psizes) = any(
    overlap_push(box_at(anchors[i], sel[i], psizes[i]),
                 box_at(anchors[j], sel[j], psizes[j])) != Vec2f(0, 0)
    for i in 1:length(anchors) for j in (i+1):length(anchors))

@testset "side_select picks the shortest-leader slot when unconflicted" begin
    # side_select minimizes (overlaps·W + ‖offset‖). With no conflict the cost is pure
    # leader length, so a wide label (30×16: half-height 8 < half-width 15) goes to T
    # (offset (0,8), leader 8) — NOT the seed's TR (leader 17). This leader-minimizing
    # override of the Imhof seed is exactly the §7e win; the seed only sets the start.
    p = RepelParams()
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(100, 100), Point2f(300, 300)]   # far apart, no conflict
    sizes   = [Vec2f(30, 16), Vec2f(30, 16)]
    ps = psz(sizes, p.box_padding)
    seed = [slot_offset(:TR, sizes[i], p.point_padding) for i in 1:2]   # Imhof-preferred start
    sel = side_select(anchors, sizes, ps, bounds, seed, p)
    @test sel[1] ≈ slot_offset(:T, sizes[1], p.point_padding)   # (0, 8): shortest in-bounds slot
    @test sel[2] ≈ slot_offset(:T, sizes[2], p.point_padding)
end

@testset "side_select resolves a head-on conflict to opposite sides" begin
    p = RepelParams()
    bounds = Rect2f(0, 0, 400, 400)
    # Two anchors close in x; if both pick TR they overlap. Greedy should split them.
    anchors = [Point2f(195, 200), Point2f(205, 200)]
    sizes   = [Vec2f(60, 16), Vec2f(60, 16)]
    ps = psz(sizes, p.box_padding)
    seed = [slot_offset(:TR, sizes[i], p.point_padding) for i in 1:2]
    sel = side_select(anchors, sizes, ps, bounds, seed, p)
    @test !anyov(anchors, sel, ps)                      # overlap term dominates → separated
end

@testset "side_select fixes pinned labels" begin
    p = RepelParams()
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(200, 200), Point2f(260, 200)]
    sizes   = [Vec2f(40, 16), Vec2f(40, 16)]
    ps = psz(sizes, p.box_padding)
    seed = [slot_offset(:TR, sizes[i], p.point_padding) for i in 1:2]
    pin = BitVector([true, false])
    pinned = [Vec2f(0, -50), Vec2f(0, 0)]               # label 1 pinned below
    sel = side_select(anchors, sizes, ps, bounds, seed, p;
                      pin_mask = pin, pinned_offsets = pinned)
    @test sel[1] == Vec2f(0, -50)                       # pinned: untouched
end

@testset "side_select avoids an obstacle" begin
    p = RepelParams()
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(200, 200)]
    sizes   = [Vec2f(30, 16)]                            # wide → unconflicted pick is T
    ps = psz(sizes, p.box_padding)
    seed = [slot_offset(:TR, sizes[1], p.point_padding)]
    # Obstacle blankets the T-slot box (centered at (200, 208)) from y=204 up, but stays
    # clear of the B-slot box (spans y[180,204], touch-only at 204 → no overlap), so the
    # greedy must move the label off T to a non-overlapping slot.
    obs = Rect2f(150, 204, 100, 60)
    sel = side_select(anchors, sizes, ps, bounds, seed, p; obstacles = [obs])
    @test overlap_push(box_at(anchors[1], sel[1], ps[1]), obs) == Vec2f(0, 0)
end

@testset "side_select only_move=:x restricts to horizontal slots" begin
    p = RepelParams(; only_move = :x)
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(100, 100)]
    sizes   = [Vec2f(30, 16)]
    ps = psz(sizes, p.box_padding)
    seed = [slot_offset(:TR, sizes[1], p.point_padding)]
    sel = side_select(anchors, sizes, ps, bounds, seed, p)
    @test sel[1][2] ≈ 0f0                                # y-component locked to 0
end

@testset "side_select is deterministic" begin
    p = RepelParams()
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(195, 200), Point2f(205, 200), Point2f(200, 215)]
    sizes   = [Vec2f(50, 16) for _ in 1:3]
    ps = psz(sizes, p.box_padding)
    seed = [slot_offset(:TR, sizes[i], p.point_padding) for i in 1:3]
    s1 = side_select(anchors, sizes, ps, bounds, seed, p)
    s2 = side_select(anchors, sizes, ps, bounds, seed, p)
    @test s1 == s2
end
