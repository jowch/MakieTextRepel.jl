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

using MakieTextRepel: solve_repel
using MakieTextRepel: box_at

# direct AABB overlap test (px), with a tolerance — the spring leaves sub-pixel
# residual overlap at equilibrium, which we treat as "separated".
function _boxes_overlap(b1, b2, tol)
    c1 = b1.origin .+ b1.widths ./ 2
    c2 = b2.origin .+ b2.widths ./ 2
    ox = (b1.widths[1] + b2.widths[1]) / 2 - abs(c1[1] - c2[1])
    oy = (b1.widths[2] + b2.widths[2]) / 2 - abs(c1[2] - c2[2])
    return ox > tol && oy > tol
end

function _any_overlap(anchors, offsets, sizes, pad; tol = 0.5)
    psizes = [s .+ Vec2f(2f0 * Float32(pad)) for s in sizes]
    boxes = [box_at(anchors[i], offsets[i], psizes[i]) for i in eachindex(anchors)]
    for i in eachindex(boxes), j in eachindex(boxes)
        i < j || continue
        _boxes_overlap(boxes[i], boxes[j], tol) && return true
    end
    return false
end

@testset "solve_repel" begin
    p = RepelParams(box_padding = 0.0)

    # empty / single
    @test solve_repel(Point2f[], Vec2f[], p) == (Vec2f[], falses(0))
    o1, d1 = solve_repel([Point2f(5, 5)], [Vec2f(10, 4)], p)
    @test o1 == [Vec2f(0, 0)]          # single label never moves
    @test d1 == falses(1)

    # two overlapping labels separate
    anchors = [Point2f(0, 0), Point2f(2, 0)]
    sizes = [Vec2f(20, 10), Vec2f(20, 10)]
    offsets, dropped = solve_repel(anchors, sizes, p)
    @test !_any_overlap(anchors, offsets, sizes, p.box_padding)
    @test !any(dropped)

    # determinism: identical inputs → identical outputs
    o_a, _ = solve_repel(anchors, sizes, p)
    o_b, _ = solve_repel(anchors, sizes, p)
    @test o_a == o_b

    # axis constraint: only_move = :x → zero y displacement
    px = RepelParams(box_padding = 0.0, only_move = :x)
    ox, _ = solve_repel(anchors, sizes, px)
    @test all(o -> o[2] == 0, ox)

    # stability: many coincident labels don't NaN
    co = fill(Point2f(0, 0), 8)
    cs = fill(Vec2f(15, 8), 8)
    oc, _ = solve_repel(co, cs, RepelParams())
    @test all(o -> all(isfinite, o), oc)
end

using MakieTextRepel: compute_drops, clamp_box_offset

@testset "compute_drops" begin
    anchors = [Point2f(0, 0), Point2f(1, 0), Point2f(2, 0)]
    # Narrow boxes (width 1.5): neighbours 1px apart overlap; the ends, 2px
    # apart, do NOT. (Wide boxes would make all three mutually overlap.)
    psizes = [Vec2f(1.5, 1.0), Vec2f(1.5, 1.0), Vec2f(1.5, 1.0)]
    offsets = [Vec2f(0, 0), Vec2f(0, 0), Vec2f(0, 0)]

    # Inf max_overlaps → nothing dropped
    @test compute_drops(anchors, offsets, psizes, Inf) == falses(3)

    # max_overlaps = 1 → the middle box overlaps both neighbours (count 2) and is
    # dropped; each end overlaps only the middle (count 1) and survives.
    dropped = compute_drops(anchors, offsets, psizes, 1)
    @test dropped == BitVector([false, true, false])
end

@testset "solve_repel clamping" begin
    bounds = Rect2f(0, 0, 200, 120)
    pad = 4.0
    # padded box of label i at its solved offset
    pbox(anchors, offsets, sizes, i) =
        box_at(anchors[i], offsets[i], sizes[i] .+ Vec2f(2f0 * Float32(pad)))

    # anchors hugging the edges/corners; every final padded box must stay inside
    anchors = [Point2f(5, 5), Point2f(195, 5), Point2f(5, 115),
               Point2f(195, 115), Point2f(100, 60)]
    sizes = fill(Vec2f(40, 16), 5)
    offsets, _ = solve_repel(anchors, sizes, RepelParams(box_padding = pad, bounds = bounds))
    for i in eachindex(anchors)
        b = pbox(anchors, offsets, sizes, i)
        @test b.origin[1] >= -1e-2
        @test b.origin[2] >= -1e-2
        @test b.origin[1] + b.widths[1] <= 200 + 1e-2
        @test b.origin[2] + b.widths[2] <= 120 + 1e-2
    end

    # bounds = nothing is the same as not passing bounds at all (clamp truly off)
    a = solve_repel(anchors, sizes, RepelParams(box_padding = pad, bounds = nothing))[1]
    b = solve_repel(anchors, sizes, RepelParams(box_padding = pad))[1]
    @test a == b

    # degenerate: label wider than bounds → pinned, finite, no NaN
    big, _ = solve_repel([Point2f(100, 60)], [Vec2f(400, 10)],
                         RepelParams(box_padding = 0.0, bounds = bounds))
    @test all(isfinite, big[1])
end
