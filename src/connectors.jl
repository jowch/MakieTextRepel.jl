# connectors.jl — pure connector-segment construction (pixel space).

"""
Build flat `Point2f` endpoint pairs for `linesegments!` (pixel space): each kept
label that moved more than `min_len` gets a segment from its anchor to the near
edge of its (padded) box. Dropped or barely-moved labels are skipped.
"""
function build_connectors(anchors::Vector{Point2f}, offsets::Vector{Vec2f},
                          sizes::Vector{Vec2f}, dropped::BitVector,
                          min_len::Real, box_padding::Real)
    segs = Point2f[]
    pad = Float32(box_padding)
    for i in eachindex(anchors)
        dropped[i] && continue
        norm(offsets[i]) <= min_len && continue
        psize = sizes[i] .+ 2pad
        box = box_at(anchors[i], offsets[i], psize)
        edge = clip_to_box_edge(box, anchors[i])
        edge === nothing && continue       # anchor strictly inside padded box
        push!(segs, anchors[i], edge)
    end
    return segs
end
