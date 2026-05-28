# test_solver.jl
using MakieTextRepel: RepelParams, init_offsets, box_at
using GeometryBasics
using LinearAlgebra

@testset "init_offsets" begin
    p = RepelParams()

    # distinct anchors → every label gets a deterministic golden-angle offset
    anchors = [Point2f(0, 0), Point2f(100, 100)]
    sizes = [Vec2f(10, 10), Vec2f(10, 10)]   # CALLER passes PADDED sizes; here pad=0 for simplicity
    o1 = init_offsets(anchors, sizes, p)
    o2 = init_offsets(anchors, sizes, p)
    @test o1 == o2                              # deterministic
    @test norm(o1[1]) > 0 && norm(o1[2]) > 0    # every label moved off (0,0)
    @test o1[1] != o1[2]                        # distinct angles per label

    # Geometric invariant: at the init offset, the anchor lies ON or OUTSIDE the
    # (passed-in, already-padded) box of the label.
    for i in eachindex(anchors)
        box = box_at(anchors[i], o1[i], sizes[i])
        c = box.origin .+ box.widths ./ 2
        d = anchors[i] .- c
        hw, hh = box.widths[1] / 2, box.widths[2] / 2
        @test abs(d[1]) >= hw - 1e-4 || abs(d[2]) >= hh - 1e-4   # on or outside
    end

    # coincident anchors → still get distinct golden-angle offsets
    co = [Point2f(0, 0), Point2f(0, 0), Point2f(0, 0)]
    cs = [Vec2f(10, 10), Vec2f(10, 10), Vec2f(10, 10)]
    occ = init_offsets(co, cs, p)
    @test occ[1] != occ[2]
    @test occ[2] != occ[3]
    @test occ[1] != occ[3]

    # zero-size label (empty string) → r_min floor produces a non-zero offset
    zero_sz = [Vec2f(0, 0)]
    zero_anchor = [Point2f(0, 0)]
    @test norm(init_offsets(zero_anchor, zero_sz, p)[1]) >= 1.0f0 - 1e-4
end

@testset "RepelParams copy-with-overrides constructor" begin
    base = RepelParams(force = (2.0, 2.0), max_iter = 100)
    overridden = RepelParams(base; max_iter = 50)
    @test overridden.force == (2.0, 2.0)         # carried over
    @test overridden.max_iter == 50              # overridden
    @test overridden.bounds === nothing          # default carried over

    bnds = Rect2f(0, 0, 100, 100)
    with_bounds = RepelParams(base; bounds = bnds)
    @test with_bounds.bounds == bnds
    @test with_bounds.force == (2.0, 2.0)        # unchanged
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
    empty_result = solve_repel(Point2f[], Vec2f[], p)
    @test empty_result.offsets == Vec2f[]
    @test empty_result.dropped == falses(0)
    @test empty_result.iter == 0
    o1, d1 = solve_repel([Point2f(5, 5)], [Vec2f(10, 4)], p)
    # Single label: own-anchor repulsion now active. The spring pulls inward from
    # init but cannot reach 0; equilibrium sits inside the init radius.
    @test norm(o1[1]) > 0
    # Init magnitude for label size (10, 4) and box_padding = 0:
    # r_init = sqrt(5^2 + 2^2) ≈ 5.39. Final |offset| must be < this (spring
    # pulled inward).
    @test norm(o1[1]) < 5.4f0
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

    # axis constraint: only_move = :y → zero x displacement. Symmetric to :x;
    # also guards the `_constrain` wrap on init_offsets — without that wrap an
    # x-component would leak from the new always-non-zero init. 20 labels
    # exercise indices where sin(i·φ_g) is small (e.g. i=17, sin ≈ 0.041) and
    # confirms own-anchor `point_push` still drives them well off the anchor
    # (|offset_y| ≥ hh-pad), refuting any "label settles on anchor under :y"
    # concern from the projection of the golden-angle init.
    yanchors = [Point2f(i * 30, 0) for i in 1:20]
    ysizes = fill(Vec2f(20, 10), 20)
    py = RepelParams(box_padding = 0.0, only_move = :y)
    oy, _ = solve_repel(yanchors, ysizes, py)
    @test all(o -> o[1] == 0, oy)               # x stays locked
    @test all(o -> abs(o[2]) >= 4.9f0, oy)      # all labels driven off anchor

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

@testset "solve_repel clamp respects only_move" begin
    # Label anchored past the top of the bounds (y), under only_move = :x. The clamp
    # must not introduce the forbidden y motion: its y-offset must match the unclamped
    # run, while x is still confined inside the bounds.
    anchors = [Point2f(195, 130)]
    sizes = [Vec2f(40, 20)]
    bounds = Rect2f(0, 0, 200, 100)
    ox  = solve_repel(anchors, sizes, RepelParams(only_move = :x, box_padding = 0.0, bounds = bounds))[1]
    oxn = solve_repel(anchors, sizes, RepelParams(only_move = :x, box_padding = 0.0, bounds = nothing))[1]
    @test abs(ox[1][2] - oxn[1][2]) < 1e-3        # clamp left the forbidden y axis alone
    bx = box_at(anchors[1], ox[1], sizes[1])
    @test bx.origin[1] >= -1e-3                   # x still confined
    @test bx.origin[1] + bx.widths[1] <= 200 + 1e-3
end

@testset "solve_repel converges under edge-crowding" begin
    # Wide labels crammed into a small box → they crowd against the walls. Without
    # step-cap cooling this settles into a period-2 limit cycle. We compare adjacent
    # iteration counts (N vs N+1) precisely because that lands on opposite phases of a
    # period-2 cycle: if it were still cycling the two would differ by the cycle
    # amplitude (measured ≈10px uncooled), whereas a settled solution differs
    # negligibly (≈0.1px). Adjacent counts catch parity cycling that a same-parity
    # mid-run pair (e.g. 2000 vs 3000) would miss.
    bounds = Rect2f(0, 0, 80, 48)   # small enough that labels crowd the walls
    anchors = [Point2f(12, 12), Point2f(45, 12), Point2f(78, 12),
               Point2f(28, 43), Point2f(62, 43)]
    sizes = fill(Vec2f(46, 18), 5)
    pa = RepelParams(box_padding = 2.0, bounds = bounds, max_iter = 3000)
    pb = RepelParams(box_padding = 2.0, bounds = bounds, max_iter = 3001)
    oa = solve_repel(anchors, sizes, pa)[1]
    ob = solve_repel(anchors, sizes, pb)[1]
    @test maximum(norm.(oa .- ob)) < 1.0   # converged, not limit-cycling
    @test all(o -> all(isfinite, o), oa)
end

@testset "solve_repel: own-anchor repulsion fans out coincident clusters" begin
    # 5+ labels at the same anchor, varying sizes. All finite, all distinct,
    # no two boxes still overlapping after solve.
    co = fill(Point2f(0, 0), 6)
    cs = [Vec2f(20, 10), Vec2f(15, 12), Vec2f(25, 8),
          Vec2f(18, 14), Vec2f(22, 9), Vec2f(16, 11)]
    offs, _ = solve_repel(co, cs, RepelParams(box_padding = 2.0))
    @test all(o -> all(isfinite, o), offs)
    @test !_any_overlap(co, offs, cs, 2.0; tol = 0.5)
    # No two offsets identical (golden-angle init guarantees distinct seeds).
    for i in 1:length(offs), j in (i+1):length(offs)
        @test offs[i] != offs[j]
    end
end

@testset "solve_repel — obstacles kwarg" begin
    # Helper: compute the label's padded bbox at its final offset, fully
    # inline so the test doesn't depend on internal helper exports.
    function _label_bbox(anchor::Point2f, offset::Vec2f, size::Vec2f, pad::Real)
        psize = size .+ 2 * Float32(pad)
        center = anchor .+ offset
        Rect2f(center[1] - psize[1]/2, center[2] - psize[2]/2,
               psize[1], psize[2])
    end
    function _overlaps(a::Rect2f, b::Rect2f)
        return !(a.origin[1] + a.widths[1] <= b.origin[1] ||
                 b.origin[1] + b.widths[1] <= a.origin[1] ||
                 a.origin[2] + a.widths[2] <= b.origin[2] ||
                 b.origin[2] + b.widths[2] <= a.origin[2])
    end

    # Two labels on either side of an obstacle. Without the obstacle they
    # settle close to their anchors; the obstacle pushes them clear.
    anchors = [Point2f(0, 50), Point2f(100, 50)]
    sizes   = [Vec2f(20, 10), Vec2f(20, 10)]
    p = RepelParams(max_iter = 500, point_padding = 0.0)
    obstacle = Rect2f(40, 40, 20, 20)   # blocks the corridor between them

    # Sanity: empty obstacles vector === no-op vs not passing obstacles at all.
    a = solve_repel(anchors, sizes, p)
    b = solve_repel(anchors, sizes, p; obstacles = Rect2f[])
    @test a.offsets == b.offsets

    # With an obstacle, neither resulting bbox overlaps it.
    c = solve_repel(anchors, sizes, p; obstacles = [obstacle])
    for i in 1:2
        bb = _label_bbox(anchors[i], c.offsets[i], sizes[i], p.box_padding)
        @test !_overlaps(bb, obstacle)
    end

    # Multiple disjoint obstacles: same invariant for each.
    obs2 = [Rect2f(40, 40, 20, 20), Rect2f(40, 10, 20, 20)]
    d = solve_repel(anchors, sizes, p; obstacles = obs2)
    for i in 1:2, o in obs2
        bb = _label_bbox(anchors[i], d.offsets[i], sizes[i], p.box_padding)
        @test !_overlaps(bb, o)
    end
end

@testset "solve_repel returns NamedTuple with diagnostics" begin
    anchors = [Point2f(0, 0), Point2f(50, 0)]
    sizes   = [Vec2f(20, 10), Vec2f(20, 10)]
    p = RepelParams(max_iter = 50)
    result = solve_repel(anchors, sizes, p)
    @test result isa NamedTuple
    @test propertynames(result) == (:offsets, :dropped, :iter, :residual)
    @test result.offsets isa Vector{Vec2f}
    @test result.dropped isa BitVector
    @test result.iter isa Int
    @test 1 <= result.iter <= 50
    @test result.residual isa Float32
    @test result.residual >= 0
end
