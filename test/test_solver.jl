# test_solver.jl
using MakieTextRepel: RepelParams, explode_init
using GeometryBasics
using LinearAlgebra

@testset "explode_init" begin
    p = RepelParams()

    # distinct anchors → no initial nudge
    anchors = [Point2f(0, 0), Point2f(100, 100)]
    sizes = [Vec2f(10, 10), Vec2f(10, 10)]
    @test explode_init(anchors, sizes, p) == [Vec2f(0, 0), Vec2f(0, 0)]

    # coincident anchors → later ones nudged off-origin, deterministically
    co = [Point2f(0, 0), Point2f(0, 0), Point2f(0, 0)]
    cs = [Vec2f(10, 10), Vec2f(10, 10), Vec2f(10, 10)]
    off1 = explode_init(co, cs, p)
    off2 = explode_init(co, cs, p)
    @test off1 == off2                 # deterministic
    @test norm(off1[2]) > 0            # second coincident label nudged
    @test norm(off1[3]) > 0
end
