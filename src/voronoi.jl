# voronoi.jl — Voronoi cell computation + cell-fit predicate.

using Random
using DelaunayTriangulation
const DT = DelaunayTriangulation

"""
True iff all four corners of `box` lie inside convex polygon `poly`.
Uses sign-of-cross-product per edge; CCW winding required. Sufficient because
Voronoi cells clipped to a convex viewport remain convex.
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
    voronoi_cells(anchors, viewport) -> Vector{Union{Polygon, Nothing}}

Voronoi cells for `anchors` clipped to `viewport`. Length equals `length(anchors)`.
Each entry is a `GeometryBasics.Polygon` (CCW exterior) or `nothing` for non-finite,
coincident, or degenerate-input anchors (fewer than three distinct finite coordinates).

Determinism: distinct coordinates sorted lexicographically; DT.jl RNG seeded
`MersenneTwister(0)`; cells mapped back via (x, y) → cell dict.
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

    # Collinear → degenerate circumcenters at ∞; bail before handing to DT.jl.
    if _all_collinear(distinct)
        return cells
    end

    # 4. Triangulate; clip Voronoi cells to viewport.
    # DT.jl can throw on inputs that slip past `_all_collinear` (near-collinear at
    # Float32, four-on-a-circle, etc.). Catch and fall back to all-nothing so every
    # label uses TR Imhof rather than crashing the compute graph.
    rng = MersenneTwister(0)
    points = [(Float64(p[1]), Float64(p[2])) for p in distinct]
    coord_to_cell = Dict{Tuple{Float32, Float32}, GeometryBasics.Polygon}()
    try
        tri = DT.triangulate(points; rng = rng)
        vor = DT.voronoi(tri; clip = true, clip_polygon = _viewport_clip(viewport))

        # 5. Build coord → cell mapping.
        # DT.jl returns CCW-wound rings, closed (first == last). Drop the closing duplicate.
        for (idx, coord) in enumerate(distinct)
            poly_pts = DT.get_polygon_coordinates(vor, idx)
            ring = [Point2f(Float32(pt[1]), Float32(pt[2])) for pt in poly_pts[1:end-1]]
            coord_to_cell[coord] = GeometryBasics.Polygon(ring)
        end
    catch _e
        return cells   # all nothing; degraded but safe
    end

    # 6. Assign cells to non-coincident finite anchors only.
    # Coincident anchors (counts[k] > 1) get nothing → TR Imhof fallback for both.
    for i in 1:n
        finite[i] || continue
        k = (anchors[i][1], anchors[i][2])
        counts[k] == 1 && (cells[i] = coord_to_cell[k])
    end

    return cells
end

"""True iff every point in `pts` lies on one line. Assumes `length(pts) ≥ 2`.
Epsilon `_COLLINEAR_EPS` (1e-3 px²) absorbs Float32 rounding. The `try`/`catch`
in `voronoi_cells` handles near-collinear inputs that slip past this guard."""
const _COLLINEAR_EPS = 1f-3   # px²; loose enough to absorb Float32 rounding at typical pixel scales

function _all_collinear(pts::Vector{Tuple{Float32, Float32}})
    n = length(pts)
    n < 3 && return true
    p1 = pts[1]
    p2 = pts[2]
    dx = p2[1] - p1[1]
    dy = p2[2] - p1[2]
    for k in 3:n
        pk = pts[k]
        # Cross of (p2-p1) and (pk-p1): ≈zero iff pk lies on line through p1,p2.
        cr = dx * (pk[2] - p1[2]) - dy * (pk[1] - p1[1])
        abs(cr) <= _COLLINEAR_EPS || return false
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
