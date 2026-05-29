using MakieTextRepel
using MakieTextRepel: box_inside_polygon
using GeometryBasics
using Test

@testset "box_inside_polygon" begin
    # CCW square (10×10) at origin
    poly = Polygon([Point2f(0, 0), Point2f(10, 0), Point2f(10, 10), Point2f(0, 10)])

    # Box entirely inside
    @test box_inside_polygon(Rect2f(2, 2, 4, 4), poly) == true

    # Box with one corner just outside
    @test box_inside_polygon(Rect2f(8, 8, 4, 4), poly) == false

    # Box entirely outside
    @test box_inside_polygon(Rect2f(20, 20, 4, 4), poly) == false

    # Triangular cell
    tri = Polygon([Point2f(0, 0), Point2f(10, 0), Point2f(0, 10)])
    @test box_inside_polygon(Rect2f(1, 1, 2, 2), tri) == true
    @test box_inside_polygon(Rect2f(5, 5, 2, 2), tri) == false  # crosses hypotenuse
end

using MakieTextRepel: voronoi_cells

@testset "voronoi_cells" begin
    viewport = Rect2f(0, 0, 100, 100)

    # n = 0 → empty
    @test voronoi_cells(Point2f[], viewport) == Union{GeometryBasics.Polygon, Nothing}[]

    # n = 1 → single nothing slot (need ≥ 3 for triangulation)
    @test voronoi_cells([Point2f(50, 50)], viewport) == [nothing]

    # n = 2 → two nothing slots
    @test voronoi_cells([Point2f(25, 50), Point2f(75, 50)], viewport) == [nothing, nothing]

    # n = 3 non-collinear → three real cells
    anchors = [Point2f(25, 25), Point2f(75, 25), Point2f(50, 75)]
    cells = voronoi_cells(anchors, viewport)
    @test length(cells) == 3
    @test all(c !== nothing for c in cells)

    # Defensive: returned polygons are CCW-wound (positive signed shoelace area).
    # If DT.jl ever changes its winding convention, box_inside_polygon silently
    # inverts; this assertion is the canary.
    for c in cells
        c === nothing && continue
        pts = decompose(Point2f, c.exterior)
        m = length(pts)
        area = 0.0
        for k in 1:m
            a = pts[k]; b = pts[k % m + 1]
            area += a[1] * b[2] - b[1] * a[2]
        end
        @test area > 0
    end

    # Coincident anchors → both nothing; the other distinct anchors still get cells.
    # (Coords are chosen non-collinear so the distinct set forms a valid triangulation.)
    anchors = [Point2f(25, 25), Point2f(25, 25), Point2f(75, 25), Point2f(50, 75)]
    cells = voronoi_cells(anchors, viewport)
    @test cells[1] === nothing
    @test cells[2] === nothing
    @test cells[3] !== nothing
    @test cells[4] !== nothing

    # NaN anchor → nothing in that slot, others fine
    anchors = [Point2f(25, 25), Point2f(75, 25), Point2f(50, 75), Point2f(NaN, NaN)]
    cells = voronoi_cells(anchors, viewport)
    @test cells[4] === nothing
    @test cells[1] !== nothing

    # Determinism: same input → same cells across two calls
    a1 = voronoi_cells([Point2f(10, 10), Point2f(50, 50), Point2f(90, 90), Point2f(10, 90)], viewport)
    a2 = voronoi_cells([Point2f(10, 10), Point2f(50, 50), Point2f(90, 90), Point2f(10, 90)], viewport)
    @test a1 == a2
end

using MakieTextRepel: slot_offset, IMHOF_ORDER

@testset "Imhof slots" begin
    p = 2.0f0
    w = 10.0f0
    h = 6.0f0

    # All 8 directions, with anchor at origin and label center at the returned offset.
    # Verify each slot positions the label so the anchor sits outside the axis-aligned
    # padded box (one edge of the box is at distance p from the anchor).
    expected = Dict(
        :TR => Vec2f(p + w/2,  p + h/2),
        :R  => Vec2f(p + w/2,  0),
        :T  => Vec2f(0,        p + h/2),
        :BR => Vec2f(p + w/2, -p - h/2),
        :L  => Vec2f(-p - w/2, 0),
        :BL => Vec2f(-p - w/2, -p - h/2),
        :B  => Vec2f(0,       -p - h/2),
        :TL => Vec2f(-p - w/2,  p + h/2),
    )
    for (slot, expect) in expected
        @test slot_offset(slot, Vec2f(w, h), p) ≈ expect
    end

    # Preference order
    @test IMHOF_ORDER == (:TR, :R, :T, :BR, :L, :BL, :B, :TL)
end

using MakieTextRepel: initial_offsets, RepelParams

@testset "initial_offsets" begin
    # Sparse: each anchor's cell easily fits a small label at TR. Expect every offset
    # to equal the TR slot.
    viewport = Rect2f(0, 0, 200, 200)
    anchors = [Point2f(50, 50), Point2f(150, 50), Point2f(100, 150)]
    sizes = [Vec2f(10, 6), Vec2f(10, 6), Vec2f(10, 6)]
    params = RepelParams(point_padding = 2.0, box_padding = 0.0)
    cells = voronoi_cells(anchors, viewport)

    offsets = initial_offsets(anchors, sizes, cells, params)
    @test length(offsets) == 3
    expected_tr = slot_offset(:TR, sizes[1], params.point_padding)
    @test offsets[1] ≈ expected_tr
    @test offsets[2] ≈ expected_tr
    @test offsets[3] ≈ expected_tr

    # n = 1: cell is nothing → TR fallback.
    cells1 = voronoi_cells([Point2f(50, 50)], viewport)
    offs1 = initial_offsets([Point2f(50, 50)], [Vec2f(10, 6)], cells1, params)
    @test offs1[1] ≈ expected_tr

    # only_move = :x → y-component zeroed in initial offset
    params_x = RepelParams(point_padding = 2.0, only_move = :x)
    offs_x = initial_offsets(anchors, sizes, cells, params_x)
    @test all(o -> o[2] == 0f0, offs_x)

    # only_move = :y → x-component zeroed
    params_y = RepelParams(point_padding = 2.0, only_move = :y)
    offs_y = initial_offsets(anchors, sizes, cells, params_y)
    @test all(o -> o[1] == 0f0, offs_y)

    # Determinism
    @test initial_offsets(anchors, sizes, cells, params) == initial_offsets(anchors, sizes, cells, params)
end

@testset "initial_offsets honors pinned indices" begin
    anchors = [Point2f(0, 0), Point2f(50, 0), Point2f(0, 50)]
    sizes   = [Vec2f(10, 6), Vec2f(10, 6), Vec2f(10, 6)]
    cells   = Union{GeometryBasics.Polygon, Nothing}[nothing, nothing, nothing]
    params  = RepelParams()

    # No pin args → identical to the bare call (byte-identity for the recipe path).
    base = MakieTextRepel.initial_offsets(anchors, sizes, cells, params)
    same = MakieTextRepel.initial_offsets(anchors, sizes, cells, params;
                                          pin_mask = nothing, pinned_offsets = Vec2f[])
    @test same == base

    # Pin index 2 at a specific offset → that index is seeded there, others unchanged.
    pin_mask = BitVector([false, true, false])
    pinned   = [Vec2f(0, 0), Vec2f(99, 99), Vec2f(0, 0)]
    out = MakieTextRepel.initial_offsets(anchors, sizes, cells, params;
                                         pin_mask = pin_mask, pinned_offsets = pinned)
    @test out[2] == Vec2f(99, 99)
    @test out[1] == base[1]
    @test out[3] == base[3]
end

@testset "initial_offsets pinned offset bypasses _constrain (only_move)" begin
    anchors = [Point2f(0,0), Point2f(50,0)]
    sizes   = [Vec2f(10,6), Vec2f(10,6)]
    cells   = Union{GeometryBasics.Polygon, Nothing}[nothing, nothing]
    params  = RepelParams(only_move = :x)
    pin_mask = BitVector([false, true])
    pinned   = [Vec2f(0,0), Vec2f(5, 7)]
    out = MakieTextRepel.initial_offsets(anchors, sizes, cells, params;
                                         pin_mask = pin_mask, pinned_offsets = pinned)
    @test out[2] == Vec2f(5, 7)          # pinned: y survives despite only_move=:x
end
