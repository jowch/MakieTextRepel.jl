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
