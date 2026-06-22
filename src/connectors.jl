# connectors.jl — per-label connector geometry + flat segment construction.

"""
Resolved connector geometry for one label. `drawn = false` when: dropped,
anchor inside padded box, trim inversion, or length below `min_segment_length`.
Field order is `(label_end, anchor_end, drawn)` — reverse of anchor→label direction.
Use `connector_for` to construct; don't call the positional constructor directly.
"""
struct Connector
    label_end::Point2f
    anchor_end::Point2f
    drawn::Bool
end

const _UNDRAWN = Connector(Point2f(0, 0), Point2f(0, 0), false)

"""
Connector geometry for one (anchor, offset, size, dropped) tuple.
Shared by `build_connectors` and `repair_crossings!`; `drawn` gates both render and scan.
"""
function connector_for(anchor::Point2f, offset::Vec2f, size::Vec2f,
                       dropped::Bool, params, min_len::Real)
    dropped && return _UNDRAWN
    pad = Float32(params.box_padding)
    ppad = Float32(params.point_padding)
    min_len_f = Float32(min_len)
    psize = size .+ 2pad
    box = box_at(anchor, offset, psize)
    edge = clip_to_box_edge(box, anchor)
    edge === nothing && return _UNDRAWN
    dir = edge .- anchor
    dlen = norm(dir)
    dlen <= ppad && return _UNDRAWN
    seg_start = anchor .+ (ppad / dlen) .* dir
    norm(edge .- seg_start) <= min_len_f && return _UNDRAWN
    return Connector(edge, Point2f(seg_start), true)
end

"""
Build flat `Point2f` endpoint pairs for `linesegments!` (pixel space).
Delegates per-label decisions to `connector_for`.
"""
function build_connectors(anchors::Vector{Point2f}, offsets::Vector{Vec2f},
                          sizes::Vector{Vec2f}, dropped::BitVector,
                          min_len::Real, box_padding::Real;
                          point_padding::Real = 0.0)
    # Transient params — only the padding fields are read.
    params = RepelParams(box_padding = box_padding, point_padding = point_padding)
    segs = Point2f[]
    for i in eachindex(anchors)
        c = connector_for(anchors[i], offsets[i], sizes[i], dropped[i], params, min_len)
        c.drawn && push!(segs, c.anchor_end, c.label_end)
    end
    return segs
end
