# connectors.jl — pure connector-segment construction (pixel space).

"""
Build flat `Point2f` endpoint pairs for `linesegments!` (pixel space): each
label gets a segment from `anchor + point_padding · û` (trimmed to leave a
visible gap at the data marker) to the near edge of its padded box. Segments
are suppressed when (a) the label is dropped, (b) the anchor lies strictly
inside the padded box (no clean segment), (c) the anchor-to-edge distance
is less than `point_padding` (trim would invert direction), or (d) the
visible segment length is below `min_len`.

`point_padding` is a keyword argument so the recipe can ship the connector
change before the recipe is rewired to forward the value (intermediate
builds stay green).
"""
function build_connectors(anchors::Vector{Point2f}, offsets::Vector{Vec2f},
                          sizes::Vector{Vec2f}, dropped::BitVector,
                          min_len::Real, box_padding::Real;
                          point_padding::Real = 0.0)
    segs = Point2f[]
    pad = Float32(box_padding)
    ppad = Float32(point_padding)
    min_len_f = Float32(min_len)
    for i in eachindex(anchors)
        dropped[i] && continue
        psize = sizes[i] .+ 2pad
        box = box_at(anchors[i], offsets[i], psize)
        edge = clip_to_box_edge(box, anchors[i])
        edge === nothing && continue       # anchor strictly inside padded box
        dir = edge .- anchors[i]
        dlen = norm(dir)
        dlen <= ppad && continue           # trim would invert direction
        seg_start = anchors[i] .+ (ppad / dlen) .* dir
        norm(edge .- seg_start) <= min_len_f && continue   # visible-length filter
        push!(segs, seg_start, edge)
    end
    return segs
end
