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
