# geometry.jl — pure axis-aligned bounding-box helpers (GeometryBasics only).

"""Sign that never returns 0 (deterministic tie-break toward +)."""
sign0(x::Real) = x >= 0 ? 1f0 : -1f0

"""Box for a label of `size` (w,h) centered at `anchor + offset`."""
box_at(anchor::Point2f, offset::Vec2f, size::Vec2f) =
    Rect2f(Point2f(anchor .+ offset .- size ./ 2), size)

_center(b::Rect2f) = Point2f(b.origin .+ b.widths ./ 2)

"""
Per-axis separation push given center-difference `d` and per-axis overlaps.
Zero if not overlapping. Pushes only along axes carrying directional info; a
perfectly-aligned axis (`d[k] == 0`) contributes 0 so an aligned pair separates
along the *other* axis instead of running away together. Fully-coincident
centers fall back to +x (explosion init normally prevents this case).
"""
function _aniso_push(d, ox::Real, oy::Real)
    (ox <= 0 || oy <= 0) && return Vec2f(0, 0)
    (d[1] == 0 && d[2] == 0) && return Vec2f(ox, 0)
    px = d[1] == 0 ? 0f0 : sign0(d[1]) * ox
    py = d[2] == 0 ? 0f0 : sign0(d[2]) * oy
    return Vec2f(px, py)
end

"""Per-axis push moving box `a` away from overlapping box `b` (zero if disjoint)."""
function overlap_push(a::Rect2f, b::Rect2f)
    d = _center(a) .- _center(b)
    ox = (a.widths[1] + b.widths[1]) / 2 - abs(d[1])
    oy = (a.widths[2] + b.widths[2]) / 2 - abs(d[2])
    return _aniso_push(d, ox, oy)
end

"""
Push box away from point `p` if `p` lies within the box expanded by `padding`.
Zero vector otherwise. Uses the same aligned-axis-safe scheme as `overlap_push`.
"""
function point_push(box::Rect2f, p::Point2f, padding::Float32)
    ex = Rect2f(Point2f(box.origin .- padding), box.widths .+ 2padding)
    d = _center(ex) .- p
    ox = ex.widths[1] / 2 - abs(d[1])
    oy = ex.widths[2] / 2 - abs(d[2])
    return _aniso_push(d, ox, oy)
end

"""
Point on the boundary of `box` along the ray from the box center toward `target`
(ggrepel-style connector attachment). Returns the center if `target` == center.
"""
function clip_to_box_edge(box::Rect2f, target::Point2f)
    c = _center(box)
    d = target .- c
    (d[1] == 0 && d[2] == 0) && return c
    hw = box.widths[1] / 2
    hh = box.widths[2] / 2
    tx = d[1] == 0 ? Inf32 : hw / abs(d[1])
    ty = d[2] == 0 ? Inf32 : hh / abs(d[2])
    t = min(tx, ty)
    return Point2f(c .+ t .* d)
end
