# voronoi.jl — Voronoi cell computation + cell-fit predicate.

using DelaunayTriangulation
const DT = DelaunayTriangulation

"""
Test whether `box`'s four corners all lie inside the convex polygon `poly`.
Uses sign-of-cross-product against each edge; consistent edge winding (CCW)
required. Sufficient for boxes inside convex Voronoi cells (which remain
convex after clipping with the convex viewport rectangle).
"""
function box_inside_polygon(box::Rect2f, poly::GeometryBasics.Polygon)
    pts = decompose(Point2f, poly.exterior)
    n = length(pts)
    n < 3 && return false
    corners = (Point2f(box.origin),
               Point2f(box.origin[1] + box.widths[1], box.origin[2]),
               Point2f(box.origin .+ box.widths),
               Point2f(box.origin[1], box.origin[2] + box.widths[2]))
    for c in corners
        for k in 1:n
            a = pts[k]
            b = pts[k % n + 1]
            # Cross of edge a→b with point a→c. CCW polygon => interior has cross > 0.
            cr = (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1])
            cr < 0 && return false
        end
    end
    return true
end
