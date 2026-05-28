# voronoi.jl — Voronoi cell computation + cell-fit predicate.

using Random
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

"""
Compute Voronoi cells for `anchors` clipped to `viewport`. Returns a vector
of length `length(anchors)` where each entry is either a `GeometryBasics.Polygon`
(the clipped cell for that anchor, with CCW exterior) or `nothing` for anchors
that are non-finite, coincident with another anchor, or part of an input with
fewer than three distinct finite anchor coordinates.

Determinism: distinct coordinates are sorted lexicographically before triangulation;
DT.jl's RNG is explicitly seeded with `MersenneTwister(0)`; cells are mapped back
to all anchors via the (x, y) → cell dictionary.
"""
function voronoi_cells(anchors::Vector{Point2f}, viewport::Rect2f)
    n = length(anchors)
    cells = Vector{Union{GeometryBasics.Polygon, Nothing}}(nothing, n)

    # 1. Identify finite anchors.
    finite = falses(n)
    for i in 1:n
        a = anchors[i]
        finite[i] = isfinite(a[1]) && isfinite(a[2])
    end

    # 2. Count coordinate occurrences (finite anchors only).
    counts = Dict{Tuple{Float32, Float32}, Int}()
    for i in 1:n
        finite[i] || continue
        k = (anchors[i][1], anchors[i][2])
        counts[k] = get(counts, k, 0) + 1
    end

    # 3. Collect distinct finite (x, y) values, sorted lex.
    distinct = sort!(collect(keys(counts)))
    length(distinct) < 3 && return cells   # all nothing

    # All distinct points collinear → degenerate triangulation (circumcenter at ∞).
    # DT.jl will warn + throw InexactError downstream; bail out cleanly instead.
    if _all_collinear(distinct)
        return cells
    end

    # 4. Triangulate distinct points; clip Voronoi cells to viewport.
    rng = MersenneTwister(0)
    points = [(Float64(p[1]), Float64(p[2])) for p in distinct]
    tri = DT.triangulate(points; rng = rng)
    vor = DT.voronoi(tri; clip = true, clip_polygon = _viewport_clip(viewport))

    # 5. Build coord → cell mapping.
    # DT.jl returns CCW-wound rings, closed (first == last). Drop the closing duplicate.
    coord_to_cell = Dict{Tuple{Float32, Float32}, GeometryBasics.Polygon}()
    for (idx, coord) in enumerate(distinct)
        poly_pts = DT.get_polygon_coordinates(vor, idx)
        ring = [Point2f(Float32(pt[1]), Float32(pt[2])) for pt in poly_pts[1:end-1]]
        coord_to_cell[coord] = GeometryBasics.Polygon(ring)
    end

    # 6. Assign cells to non-coincident finite anchors only.
    # An anchor with `counts[k] > 1` is coincident with at least one other label —
    # leave its cell as `nothing` so both labels fall through to TR Imhof fallback.
    for i in 1:n
        finite[i] || continue
        k = (anchors[i][1], anchors[i][2])
        counts[k] == 1 && (cells[i] = coord_to_cell[k])
    end

    return cells
end

"""Return `true` if every point in `pts` lies on a single line. Assumes `length(pts) ≥ 2`.
Used to bail out before handing DT.jl a degenerate point set (which it would warn
about and then throw on during Voronoi clipping)."""
function _all_collinear(pts::Vector{Tuple{Float32, Float32}})
    n = length(pts)
    n < 3 && return true
    p1 = pts[1]
    p2 = pts[2]
    dx = p2[1] - p1[1]
    dy = p2[2] - p1[2]
    for k in 3:n
        pk = pts[k]
        # Cross of (p2-p1) and (pk-p1): zero iff pk lies on line through p1,p2.
        cr = dx * (pk[2] - p1[2]) - dy * (pk[1] - p1[1])
        cr == 0 || return false
    end
    return true
end

"""DT.jl's `clip_polygon` requires `(points, boundary_nodes)` CCW Tuple form."""
function _viewport_clip(viewport::Rect2f)
    o = viewport.origin
    w = viewport.widths
    pts = [(Float64(o[1]),        Float64(o[2])),
           (Float64(o[1] + w[1]), Float64(o[2])),
           (Float64(o[1] + w[1]), Float64(o[2] + w[2])),
           (Float64(o[1]),        Float64(o[2] + w[2]))]
    return (pts, [1, 2, 3, 4, 1])
end
