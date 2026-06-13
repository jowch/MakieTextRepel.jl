using MakieTextRepel
using MakieTextRepel: ProjectionSolver, ForceSolver, solve_cluster, drop_most_overlapped!,
                      box_at, overlap_push, label_cost, RepelParams
using GeometryBasics
using Test

# deterministic stand-in for randn so the suite is rng-free and reproducible.
# Defined first because @testset bodies execute top-to-bottom as the file loads.
randn_stub(i) = sin(Float64(i) * 12.9898) * 4.0

psz(sizes, bp) = [s .+ Vec2f(2bp, 2bp) for s in sizes]
function novl(anchors, offsets, sizes, bp; dropped = nothing)
    ps = psz(sizes, bp); n = length(anchors); c = 0
    for i in 1:n, j in (i+1):n
        (dropped !== nothing && (dropped[i] || dropped[j])) && continue
        ox = (ps[i][1]+ps[j][1])/2 - abs((anchors[i][1]+offsets[i][1]) - (anchors[j][1]+offsets[j][1]))
        oy = (ps[i][2]+ps[j][2])/2 - abs((anchors[i][2]+offsets[i][2]) - (anchors[j][2]+offsets[j][2]))
        (ox > 0.5 && oy > 0.5) && (c += 1)
    end
    c
end
mean_leader(offsets, dropped) = begin
    act = [i for i in eachindex(offsets) if !dropped[i]]
    isempty(act) ? 0.0 : sum(sqrt(Float64(offsets[i][1])^2 + offsets[i][2]^2) for i in act)/length(act)
end

# Deterministic per-index jitter (no RNG) so the knot fixture is reproducible.
jitter(i::Int) = Float32(sin(12.9898 * i) * 43758.5453 % 1.0)
# Frozen leader-length regression gate. Set from the first green run of the Q battery below.
const BASELINE_SUM_LEADER = 59.896531105041504   # frozen from first green run of Q battery

@testset "ProjectionSolver: zero overlap per scene; shorter leader in aggregate" begin
    # Per-scene leader length is NOT a theorem (a sparse scene's force_pull can hug the
    # anchor closer than a discrete Imhof slot). §7e validated the win in AGGREGATE
    # (~2× shorter across knot/collinear/sparse). So assert zero-overlap per scene
    # (the hard guarantee) and the leader win as an aggregate with a real margin.
    p = RepelParams()
    fixtures = [
        ("knot",      Rect2f(0, 0, 380, 340),
         [Point2f(90 + 8randn_stub(i), 250 + 8randn_stub(i+1)) for i in 1:12], Vec2f(40, 15)),
        ("collinear", Rect2f(0, 0, 380, 340),
         [Point2f(40 + 30i, 180) for i in 0:9], Vec2f(40, 15)),
        ("scatter",   Rect2f(0, 0, 380, 340),
         [Point2f(190 + 60randn_stub(i), 170 + 50randn_stub(i+2)) for i in 1:18], Vec2f(35, 15)),
    ]
    proj_total = 0.0; force_total = 0.0
    for (name, bounds, anchors, sz) in fixtures
        sizes = [sz for _ in 1:length(anchors)]
        rp = solve_cluster(ProjectionSolver(p), anchors, sizes, bounds)
        rf = solve_cluster(ForceSolver(p), anchors, sizes, bounds)
        @test novl(anchors, rp.offsets, sizes, p.box_padding; dropped = rp.dropped) == 0  # hard, per scene
        proj_total  += mean_leader(rp.offsets, rp.dropped)
        force_total += mean_leader(rf.offsets, rf.dropped)
    end
    @test proj_total ≤ 0.9 * force_total      # aggregate leader win, generous margin
end

@testset "ProjectionSolver: over-capacity drops, survivors are overlap-free" begin
    p = RepelParams()
    bounds = Rect2f(0, 0, 80, 80)                       # tiny bounds
    anchors = [Point2f(40, 40) for _ in 1:8]
    sizes   = [Vec2f(40, 30) for _ in 1:8]              # can't all fit
    r = solve_cluster(ProjectionSolver(p), anchors, sizes, bounds)
    @test count(r.dropped) > 0
    @test novl(anchors, r.offsets, sizes, p.box_padding; dropped = r.dropped) == 0
end

@testset "ProjectionSolver: warm-start (init_state) legalizes without re-siding" begin
    p = RepelParams()
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(200, 200), Point2f(200, 200)]
    sizes   = [Vec2f(40, 40), Vec2f(40, 40)]
    # pre-separated init → must come back essentially unchanged
    init = [Vec2f(-60, 0), Vec2f(60, 0)]
    r = solve_cluster(ProjectionSolver(p), anchors, sizes, bounds; init_state = init)
    @test r.offsets[1] ≈ init[1] atol=0.5
    @test r.offsets[2] ≈ init[2] atol=0.5
    # overlapping init → separated, but offsets stay near the supplied init (no slot search)
    init2 = [Vec2f(0, 0), Vec2f(6, 0)]
    r2 = solve_cluster(ProjectionSolver(p), anchors, sizes, bounds; init_state = init2)
    @test novl(anchors, r2.offsets, sizes, p.box_padding) == 0
end

@testset "ProjectionSolver: pinned offsets are preserved bit-identically" begin
    p = RepelParams()
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(150, 200), Point2f(250, 200)]
    sizes   = [Vec2f(40, 20), Vec2f(40, 20)]
    pin = BitVector([true, false])
    pinned = [Vec2f(0, -70), Vec2f(0, 0)]
    r = solve_cluster(ProjectionSolver(p), anchors, sizes, bounds;
                      pin_mask = pin, pinned_offsets = pinned)
    @test r.offsets[1] == Vec2f(0, -70)
end

@testset "ProjectionSolver: deterministic" begin
    p = RepelParams()
    bounds = Rect2f(0, 0, 380, 340)
    anchors = [Point2f(100 + 5i, 150 + 3i) for i in 1:10]
    sizes   = [Vec2f(35, 15) for _ in 1:10]
    r1 = solve_cluster(ProjectionSolver(p), anchors, sizes, bounds)
    r2 = solve_cluster(ProjectionSolver(p), anchors, sizes, bounds)
    @test r1.offsets == r2.offsets
    @test r1.dropped == r2.dropped
end

@testset "drop_most_overlapped! picks the busiest box, ties by highest index" begin
    p = RepelParams()
    anchors = [Point2f(0, 0), Point2f(0, 0), Point2f(0, 0)]
    sizes   = [Vec2f(20, 20) for _ in 1:3]
    ps = psz(sizes, p.box_padding)
    offsets = [Vec2f(0, 0), Vec2f(0, 0), Vec2f(0, 0)]   # all coincident → all overlap all
    dropped = falses(3)
    idx = drop_most_overlapped!(dropped, anchors, offsets, ps, nothing)
    @test idx == 3                                       # tie (each overlaps 2) → highest index
    @test dropped[3]
end

@testset "drop_most_overlapped! counts obstacle overlaps" begin
    p = RepelParams()
    # Two labels far apart → zero label↔label overlap. Label 1 sits on an obstacle,
    # label 2 is clear. Obstacle-counting must drop label 1 (ov=1), NOT the highest
    # index (label 2, ov=0) that the tie-break fallback would pick if obstacles were ignored.
    anchors = [Point2f(50, 50), Point2f(300, 300)]
    sizes   = [Vec2f(20, 20) for _ in 1:2]
    ps = psz(sizes, p.box_padding)
    offsets = [Vec2f(0, 0), Vec2f(0, 0)]
    obstacles = [Rect2f(40, 40, 20, 20)]                 # covers label 1's box
    dropped = falses(2)
    idx = drop_most_overlapped!(dropped, anchors, offsets, ps, nothing, obstacles)
    @test idx == 1
    @test dropped[1]
end

@testset "ProjectionSolver: all-pinned input is returned untouched" begin
    p = RepelParams()
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(200, 200), Point2f(200, 200), Point2f(200, 200)]
    sizes   = [Vec2f(40, 20) for _ in 1:3]
    pin     = trues(3)
    # non-overlapping pin positions: boxes fan T / R / B, well clear of each other
    pinned  = [Vec2f(0, -60), Vec2f(80, 0), Vec2f(0, 60)]
    r = solve_cluster(ProjectionSolver(p), anchors, sizes, bounds;
                      pin_mask = pin, pinned_offsets = pinned)
    @test r.offsets == pinned           # all pinned → idx==0 path; offsets preserved
    @test count(r.dropped) == 0
    @test novl(anchors, r.offsets, sizes, p.box_padding; dropped = r.dropped) == 0
end

@testset "ProjectionSolver: single label is trivially placed" begin
    p = RepelParams()
    bounds = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(200, 200)]
    sizes   = [Vec2f(40, 20)]
    r = solve_cluster(ProjectionSolver(p), anchors, sizes, bounds)
    @test count(r.dropped) == 0
    @test all(isfinite, r.offsets[1])
    @test all(abs.(r.offsets[1]) .< 400)        # sane range, comfortably inside bounds
end

@testset "Q battery: marker-free, overlap-free post-legalize; crossing/leader baselines" begin
    using MakieTextRepel: ProjectionSolver, solve_cluster, label_cost, RepelParams
    fixtures = [
        ("sparse",  Rect2f(0, 0, 400, 400), [Point2f(80 + 60i, 200 + 20*(-1)^i) for i in 1:4],
                    [Vec2f(50, 18) for _ in 1:4]),
        ("knot",    Rect2f(0, 0, 300, 300), [Point2f(150 + 4jitter(i), 150 + 4jitter(i+10)) for i in 1:5],
                    [Vec2f(46, 16) for _ in 1:5]),
        ("collin",  Rect2f(0, 0, 500, 120), [Point2f(60 + 70i, 60) for i in 1:6],
                    [Vec2f(48, 16) for _ in 1:6]),
    ]
    total_overlaps = 0; total_point = 0; total_crossings = 0; total_leader = 0.0
    for (name, bounds, anchors, sizes) in fixtures
        params = RepelParams(box_padding = 4.0, point_padding = 2.0, min_segment_length = 4.0)
        offs, dropped, _, _ = solve_cluster(ProjectionSolver(params), anchors, sizes, bounds)
        q = label_cost(anchors, sizes; offsets = offs, bounds = bounds, dropped = dropped,
                       box_padding = params.box_padding, point_padding = params.point_padding,
                       min_segment_length = params.min_segment_length)
        @test q.overlaps == 0                       # zero-overlap guarantee (feasible fixtures)
        @test q.point_overlaps == 0                 # markers cleared AFTER legalize
        total_overlaps += q.overlaps; total_point += q.point_overlaps
        total_crossings += q.crossings; total_leader += q.mean_leader
    end
    # To re-derive the frozen baseline if fixtures change: print `total_leader` here and
    # update `BASELINE_SUM_LEADER` above (kept out of the default run to keep CI output clean).
    @test total_overlaps == 0 && total_point == 0   # aggregate hard-term non-regression
    @test total_crossings == 0                      # crossing baseline (Stage 3 must not regress)
    @test total_leader ≤ 1.05 * BASELINE_SUM_LEADER  # leader-length regression gate
end

@testset "ProjectionSolver: swap search untangles a warm-start crossing" begin
    using MakieTextRepel: ProjectionSolver, solve_cluster, label_cost, RepelParams
    bounds  = Rect2f(0, 0, 400, 300)
    anchors = [Point2f(100, 100), Point2f(200, 100)]
    sizes   = [Vec2f(50, 18), Vec2f(50, 18)]
    params  = RepelParams(box_padding = 4.0, point_padding = 2.0, min_segment_length = 4.0)
    # label 1 (left anchor) placed up-RIGHT, label 2 (right anchor) up-LEFT → leaders cross.
    # Boxes are 60px apart in x (58px wide padded) so they don't overlap; legalize leaves them,
    # the crossing persists into Part B, and a single offset swap untangles it.
    cross_init = [Vec2f(80, 30), Vec2f(-80, 30)]
    res = solve_cluster(ProjectionSolver(params), anchors, sizes, bounds; init_state = cross_init)
    q = label_cost(anchors, sizes; offsets = res.offsets, bounds = bounds, dropped = res.dropped,
                   box_padding = params.box_padding, point_padding = params.point_padding,
                   min_segment_length = params.min_segment_length)
    @test q.crossings == 0          # swap search untangled the warm-start crossing
    @test q.overlaps == 0           # zero-overlap guarantee survives the swap search
end

@testset "ProjectionSolver: swap search is deterministic and terminates" begin
    using MakieTextRepel: ProjectionSolver, solve_cluster, label_cost, RepelParams
    bounds  = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(100 + 7i, 200 + 11*(-1)^i) for i in 1:8]
    sizes   = [Vec2f(44, 16) for _ in 1:8]
    params  = RepelParams(box_padding = 4.0, point_padding = 2.0, min_segment_length = 4.0)
    a = solve_cluster(ProjectionSolver(params), anchors, sizes, bounds).offsets   # returns ⇒ terminated
    b = solve_cluster(ProjectionSolver(params), anchors, sizes, bounds).offsets
    @test a == b
    # over-capacity warm-start: many crossing labels in tight bounds must still terminate
    tight = Rect2f(0, 0, 120, 120)
    init  = [Vec2f(40 * cos(i), 40 * sin(i)) for i in 1:6]
    r = solve_cluster(ProjectionSolver(params), [Point2f(60, 60) for _ in 1:6],
                      [Vec2f(40, 16) for _ in 1:6], tight; init_state = init)
    @test r.offsets isa Vector{Vec2f}            # completed within UNCROSS_ROUNDS, no hang
end

@testset "ProjectionSolver: over-capacity scene stops at a fixpoint, stays overlap-free" begin
    using MakieTextRepel: ProjectionSolver, solve_cluster, label_cost, RepelParams
    bounds  = Rect2f(0, 0, 80, 80)                 # tiny bounds, 8 coincident anchors
    anchors = [Point2f(40, 40) for _ in 1:8]
    sizes   = [Vec2f(30, 14) for _ in 1:8]
    params  = RepelParams(box_padding = 4.0, point_padding = 2.0, min_segment_length = 4.0)
    res = solve_cluster(ProjectionSolver(params), anchors, sizes, bounds)
    q = label_cost(anchors, sizes; offsets = res.offsets, bounds = bounds, dropped = res.dropped,
                   box_padding = params.box_padding, point_padding = params.point_padding,
                   min_segment_length = params.min_segment_length)
    @test q.overlaps == 0                          # survivors overlap-free (zero-overlap wins conflicts)
    @test any(res.dropped)                         # over-capacity ⇒ at least one drop, no hang
end
