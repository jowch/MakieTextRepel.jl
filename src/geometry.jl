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

# Per-axis corrective shift to bring one interval inside another, preserving width.
# If the box is wider than the bounds on this axis, pin it to the lower edge.
function _clamp_axis(lo, hi, blo, bhi, w, bw)
    w  > bw  && return blo - lo   # larger than bounds → pin lower edge
    lo < blo && return blo - lo   # over lower edge → push toward +
    hi > bhi && return bhi - hi   # over upper edge → push toward -
    return 0f0
end

"""
Minimal shift to bring `box` fully inside `bounds`, preserving its size. Returns a
zero vector if it already fits. If `box` is larger than `bounds` on an axis, pins it
to that axis's lower edge — in Makie's y-up pixel space that is the left edge on x and
the bottom edge on y.
"""
function clamp_box_offset(box::Rect2f, bounds::Rect2f)
    lo, hi   = box.origin, box.origin .+ box.widths
    blo, bhi = bounds.origin, bounds.origin .+ bounds.widths
    sx = _clamp_axis(lo[1], hi[1], blo[1], bhi[1], box.widths[1], bounds.widths[1])
    sy = _clamp_axis(lo[2], hi[2], blo[2], bhi[2], box.widths[2], bounds.widths[2])
    return Vec2f(sx, sy)
end
