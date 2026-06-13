using MakieTextRepel
using MakieTextRepel: legalize, dykstra!
using GeometryBasics
using Test

# helper: count >0.5px overlapping pairs among movable+fixed boxes
function novl(anchors, offsets, psizes)
    n = length(anchors); c = 0
    for i in 1:n, j in (i+1):n
        cx = (anchors[i][1]+offsets[i][1]) - (anchors[j][1]+offsets[j][1])
        cy = (anchors[i][2]+offsets[i][2]) - (anchors[j][2]+offsets[j][2])
        ox = (psizes[i][1]+psizes[j][1])/2 - abs(cx)
        oy = (psizes[i][2]+psizes[j][2])/2 - abs(cy)
        (ox > 0.5 && oy > 0.5) && (c += 1)
    end
    c
end

@testset "dykstra! single constraint splits the move" begin
    pos = Float64[0.0, 1.0]                 # need pos[2]-pos[1] >= 4
    dykstra!(pos, [(1, 2, 4.0)], Bool[true, true])
    @test pos[2] - pos[1] ≈ 4.0 atol=1e-2
    @test pos[1] ≈ -1.5 atol=1e-2           # moved symmetrically: gap 3 → split 1.5 each
    @test pos[2] ≈ 2.5  atol=1e-2
end

@testset "dykstra! fixed endpoint absorbs nothing" begin
    pos = Float64[0.0, 1.0]
    dykstra!(pos, [(1, 2, 4.0)], Bool[false, true])  # lo fixed → only hi moves
    @test pos[1] ≈ 0.0 atol=1e-9
    @test pos[2] ≈ 4.0 atol=1e-2
end

@testset "legalize reaches zero overlap on a feasible cluster" begin
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(200, 200) for _ in 1:6]
    psizes  = [Vec2f(40, 20) for _ in 1:6]
    offsets = [Vec2f(3i, 2i) for i in 1:6]            # heavily overlapping fan
    fixed = falses(6)
    r = legalize(anchors, offsets, psizes, bounds; fixed = fixed)
    @test r.residual ≤ 0.5f0
    @test novl(anchors, r.offsets, psizes) == 0
end

@testset "legalize signals over-capacity when infeasible" begin
    # 9 boxes of 60×60 cannot fit overlap-free in 100×100 bounds.
    bounds = Rect2f(0, 0, 100, 100)
    anchors = [Point2f(50, 50) for _ in 1:9]
    psizes  = [Vec2f(60, 60) for _ in 1:9]
    offsets = [Vec2f(0, 0) for _ in 1:9]
    r = legalize(anchors, offsets, psizes, bounds; fixed = falses(9), rounds = 200)
    @test r.residual > 0.5f0                          # cannot clear → over-capacity
end

@testset "legalize holds fixed nodes still" begin
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(200, 200), Point2f(200, 200)]
    psizes  = [Vec2f(40, 40), Vec2f(40, 40)]
    offsets = [Vec2f(0, 0), Vec2f(5, 0)]              # overlapping
    fixed = BitVector([true, false])                  # label 1 pinned
    r = legalize(anchors, offsets, psizes, bounds; fixed = fixed)
    @test r.offsets[1] == Vec2f(0, 0)                 # bit-identical: fixed node never moved
    @test novl(anchors, r.offsets, psizes) == 0       # label 2 absorbed the separation
end

@testset "legalize only_move=:x leaves y untouched" begin
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(200, 200), Point2f(200, 200)]
    psizes  = [Vec2f(40, 40), Vec2f(40, 40)]
    offsets = [Vec2f(0, 0), Vec2f(5, 3)]
    r = legalize(anchors, offsets, psizes, bounds; fixed = falses(2), only_move = :x)
    @test (anchors[1][2] + r.offsets[1][2]) ≈ 200.0 atol=1e-4   # y centers unchanged
    @test (anchors[2][2] + r.offsets[2][2]) ≈ 203.0 atol=1e-4
end

@testset "legalize is a no-op (bit-identical) on a separated layout" begin
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(100, 100), Point2f(300, 300)]
    psizes  = [Vec2f(40, 40), Vec2f(40, 40)]
    offsets = [Vec2f(0, 0), Vec2f(0, 0)]
    r = legalize(anchors, offsets, psizes, bounds; fixed = falses(2))
    @test r.rounds_used == 0
    @test r.offsets[1] === offsets[1]                 # exact same Vec2f preserved
    @test r.offsets[2] === offsets[2]
end

@testset "legalize is deterministic" begin
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(200, 200) for _ in 1:8]
    psizes  = [Vec2f(30, 18) for _ in 1:8]
    offsets = [Vec2f(2i, i) for i in 1:8]
    r1 = legalize(anchors, offsets, psizes, bounds; fixed = falses(8))
    r2 = legalize(anchors, offsets, psizes, bounds; fixed = falses(8))
    @test r1.offsets == r2.offsets
end

@testset "legalize confines an out-of-bounds non-overlapping layout" begin
    bounds = Rect2f(0, 0, 200, 200)
    anchors = [Point2f(10, 100), Point2f(150, 100)]
    psizes  = [Vec2f(40, 30), Vec2f(40, 30)]
    offsets = [Vec2f(-30, 0), Vec2f(0, 0)]            # box 1 origin.x = -40, no overlap with box 2
    r = legalize(anchors, offsets, psizes, bounds; fixed = falses(2))
    for i in 1:2
        c = anchors[i] .+ r.offsets[i]
        @test c[1] - psizes[i][1]/2 >= -0.5           # left edge inside bounds
        @test c[1] + psizes[i][1]/2 <= 200 + 0.5      # right edge inside bounds
        @test c[2] - psizes[i][2]/2 >= -0.5
        @test c[2] + psizes[i][2]/2 <= 200 + 0.5
    end
    @test novl(anchors, r.offsets, psizes) == 0       # still overlap-free
    @test r.rounds_used == 0                          # top-of-round clamp confines it → breaks at round 1
end

@testset "soft keep-out nodes (#21)" begin
    using MakieTextRepel: legalize
    bounds = Rect2f(0, 0, 200, 200)

    # (a) A fixed node with NEGATIVE half-extent still separates a movable box.
    anchors = [Point2f(40, 100), Point2f(60, 100)]
    offsets = [Vec2f(0, 0), Vec2f(0, 0)]
    psizes  = [Vec2f(40, 20), Vec2f(2, 2)]      # label vs a tiny (hw=1) fixed node
    fixed   = BitVector([false, true])
    r = legalize(anchors, offsets, psizes, bounds; fixed = fixed)
    c1 = anchors[1][1] + r.offsets[1][1]
    @test abs(c1 - 60) ≥ 21 - 0.05

    # (b) A SOFT fixed node's residual is excluded, with a non-soft negative control.
    # only_move=:x + right-bound clamp ⇒ separation is genuinely unsatisfiable in-bounds
    # (the label cannot escape on y). Without the lock the label slides down on y and
    # residual→0, making the control vacuous — caught in plan review.
    bsm     = Rect2f(0, 0, 100, 200)
    anc     = [Point2f(95, 100), Point2f(70, 100)]
    off     = [Vec2f(0, 0), Vec2f(0, 0)]
    psz     = [Vec2f(40, 20), Vec2f(40, 20)]
    fx      = BitVector([false, true])
    rsoft   = legalize(anc, off, psz, bsm; fixed = fx, soft = BitVector([false, true]),
                       only_move = :x)
    rhard   = legalize(anc, off, psz, bsm; fixed = fx, only_move = :x)  # soft = none
    @test rsoft.offsets[1] != Vec2f(0, 0)        # push still fired
    @test rsoft.residual ≤ 0.5f0                 # soft pair excluded from residual
    @test rhard.residual > 0.5f0                 # negative control: counted when not soft

    # (c) point_padding=0 ⇒ mc=-box_padding ⇒ gap=unpadded_half: clean slot bit-identical.
    anc2 = [Point2f(40, 100), Point2f(40, 100)]
    off2 = [Vec2f(20, 0), Vec2f(0, 0)]
    psz2 = [Vec2f(48, 28), Vec2f(-8, -8)]         # mc=-4 ⇒ width 2*mc = -8
    fx2  = BitVector([false, true])
    r2   = legalize(anc2, off2, psz2, bounds; fixed = fx2, soft = BitVector([false, true]))
    @test r2.offsets[1] == Vec2f(20, 0)           # bit-identical: no push
end
