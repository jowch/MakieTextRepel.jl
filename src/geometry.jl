# geometry.jl — pure AABB helpers (GeometryBasics only).

"""Sign that never returns 0 (ties go to +)."""
sign0(x::Real) = x >= 0 ? 1f0 : -1f0

"""Box of `size` (w,h) centered at `anchor + offset`."""
box_at(anchor::Point2f, offset::Vec2f, size::Vec2f) =
    Rect2f(Point2f(anchor .+ offset .- size ./ 2), size)

_center(b::Rect2f) = Point2f(b.origin .+ b.widths ./ 2)

"""
Per-axis separation push given center-difference `d` and per-axis overlaps.
Zero if not overlapping. An aligned axis (`d[k] == 0`) contributes 0, so a
collinear pair separates along the other axis. Fully-coincident centers fall
back to +x.
"""
function _aniso_push(d, ox::Real, oy::Real)
    (ox <= 0 || oy <= 0) && return Vec2f(0, 0)
    (d[1] == 0 && d[2] == 0) && return Vec2f(ox, 0)
    px = d[1] == 0 ? 0f0 : sign0(d[1]) * ox
    py = d[2] == 0 ? 0f0 : sign0(d[2]) * oy
    return Vec2f(px, py)
end

"""Per-axis push moving box `a` away from overlapping box `b`. Zero if disjoint."""
function overlap_push(a::Rect2f, b::Rect2f)
    d = _center(a) .- _center(b)
    ox = (a.widths[1] + b.widths[1]) / 2 - abs(d[1])
    oy = (a.widths[2] + b.widths[2]) / 2 - abs(d[2])
    return _aniso_push(d, ox, oy)
end

"""`true` if boxes `a` and `b` overlap (i.e. `overlap_push` is nonzero)."""
boxes_overlap(a::Rect2f, b::Rect2f) = overlap_push(a, b) != Vec2f(0, 0)

"""
Push box away from point `p` if `p` lies within the box expanded by `padding`.
Zero otherwise. Same aligned-axis-safe scheme as `overlap_push`.
"""
function point_push(box::Rect2f, p::Point2f, padding::Float32)
    ex = Rect2f(Point2f(box.origin .- padding), box.widths .+ 2padding)
    d = _center(ex) .- p
    ox = ex.widths[1] / 2 - abs(d[1])
    oy = ex.widths[2] / 2 - abs(d[2])
    return _aniso_push(d, ox, oy)
end

"""
True iff point `p` lies strictly inside `box` expanded by `padding` on every side.
Strict inequalities: a point on the expanded edge is not covered. Shared by
`side_select`'s marker-avoidance term and `label_cost`'s `point_overlaps` count.
"""
function point_covered(p::Point2f, box::Rect2f, padding::Real)
    pad = Float32(padding)
    lo = box.origin .- pad
    hi = box.origin .+ box.widths .+ pad
    return p[1] > lo[1] && p[1] < hi[1] && p[2] > lo[2] && p[2] < hi[2]
end

"""
Point on `box`'s boundary along the ray from box center toward `target`
(ggrepel-style connector attachment). Returns `nothing` when `target` lies
strictly inside the box on both axes. A target on a face or corner is valid
(`t = 1`).
"""
function clip_to_box_edge(box::Rect2f, target::Point2f)
    c = _center(box)
    d = target .- c
    hw = box.widths[1] / 2
    hh = box.widths[2] / 2
    # strict-inside: a target on the boundary is still a valid endpoint
    (abs(d[1]) < hw && abs(d[2]) < hh) && return nothing
    tx = d[1] == 0 ? Inf32 : hw / abs(d[1])
    ty = d[2] == 0 ? Inf32 : hh / abs(d[2])
    t = clamp(min(tx, ty), 0f0, 1f0)   # defensive; strict-inside already guards
    return Point2f(c .+ t .* d)
end

# Per-axis corrective shift to bring one interval inside another, preserving width.
# Box wider than bounds on this axis → pin to lower edge.
function _clamp_axis(lo, hi, blo, bhi, w, bw)
    w  > bw  && return blo - lo   # larger than bounds → pin lower edge
    lo < blo && return blo - lo   # over lower edge → push toward +
    hi > bhi && return bhi - hi   # over upper edge → push toward -
    return 0f0
end

"""
Minimal shift to bring `box` fully inside `bounds`, preserving size. Returns zero
vector if already fits. If `box` is larger than `bounds` on an axis, pins to that
axis's lower edge (left on x, bottom on y in Makie's y-up pixel space).
"""
function clamp_box_offset(box::Rect2f, bounds::Rect2f)
    lo, hi   = box.origin, box.origin .+ box.widths
    blo, bhi = bounds.origin, bounds.origin .+ bounds.widths
    sx = _clamp_axis(lo[1], hi[1], blo[1], bhi[1], box.widths[1], bounds.widths[1])
    sy = _clamp_axis(lo[2], hi[2], blo[2], bhi[2], box.widths[2], bounds.widths[2])
    return Vec2f(sx, sy)
end
