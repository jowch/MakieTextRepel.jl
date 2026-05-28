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
