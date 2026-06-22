# crossings.jl — leader-segment crossing detection and 2-opt repair.

"""
Segment-segment intersection via signed-area orientation.
`true` only when endpoints of each segment strictly straddle the other line.
Endpoint-touching and collinear-overlap return `false`.
"""
function segments_cross(p1::Point2f, p2::Point2f, p3::Point2f, p4::Point2f)
    o(a, b, c) = (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1])
    d1 = o(p3, p4, p1)
    d2 = o(p3, p4, p2)
    d3 = o(p1, p2, p3)
    d4 = o(p1, p2, p4)
    return (d1 > 0 && d2 < 0 || d1 < 0 && d2 > 0) &&
           (d3 > 0 && d4 < 0 || d3 < 0 && d4 > 0)
end

"""
Pairwise O(n²) scan over `connectors`. Returns lex-ordered `(i, j)` index
pairs with `i < j` for every pair whose segments strictly cross.
Undrawn connectors (`drawn == false`) are skipped.
"""
function find_crossings(connectors::Vector{Connector})
    crossings = Tuple{Int,Int}[]
    n = length(connectors)
    for i in 1:n
        connectors[i].drawn || continue
        for j in (i+1):n
            connectors[j].drawn || continue
            if segments_cross(connectors[i].anchor_end, connectors[i].label_end,
                              connectors[j].anchor_end, connectors[j].label_end)
                push!(crossings, (i, j))
            end
        end
    end
    return crossings
end

"""
Swap the absolute positions of labels `i` and `j`, preserving label→anchor identity.
Each label's offset is recomputed so it lands at the other's prior position.
"""
function swap_positions!(offsets::Vector{Vec2f}, anchors::Vector{Point2f}, i::Int, j::Int)
    pos_i_old = anchors[i] .+ offsets[i]
    pos_j_old = anchors[j] .+ offsets[j]
    offsets[i] = Vec2f(pos_j_old .- anchors[i])
    offsets[j] = Vec2f(pos_i_old .- anchors[j])
    return offsets
end

"""
2-opt crossing repair: scan, swap non-conflicting crossing pairs, repeat until
no crossings remain or `max_iter` is hit. Returns outer iterations consumed
(0 if already crossing-free).

`min_len` must match `min_segment_length` from the recipe — no default enforces
agreement with `build_connectors`. Crossings touching a pinned label (`pin_mask`)
are skipped; early exit when no swap is possible. On cap-out, final rescan
emits a `@warn` listing residual crossings (best-effort with backstop).
"""
function repair_crossings!(offsets::Vector{Vec2f}, anchors::Vector{Point2f},
                           sizes::Vector{Vec2f}, dropped::BitVector,
                           params; min_len::Real, max_iter::Int = 100,
                           pin_mask::Union{Nothing,BitVector} = nothing)
    if pin_mask !== nothing && length(pin_mask) != length(offsets)
        throw(DimensionMismatch("pin_mask length ($(length(pin_mask))) must match offsets length ($(length(offsets)))"))
    end
    for iter in 1:max_iter
        connectors = [connector_for(anchors[i], offsets[i], sizes[i], dropped[i], params, min_len)
                      for i in eachindex(offsets)]
        crossings = find_crossings(connectors)
        isempty(crossings) && return iter - 1

        swapped = Set{Int}()
        for (i, j) in crossings
            (i in swapped || j in swapped) && continue
            pin_mask !== nothing && (pin_mask[i] || pin_mask[j]) && continue   # never move a pinned label
            swap_positions!(offsets, anchors, i, j)
            push!(swapped, i)
            push!(swapped, j)
        end
        # All remaining crossings touch pinned labels — no swap possible. Return
        # early to avoid the spurious cap-out @warn; residual is expected here.
        isempty(swapped) && return iter
    end
    # Final rescan: warn only if crossings remain (not if the last swap converged).
    final_connectors = [connector_for(anchors[i], offsets[i], sizes[i], dropped[i], params, min_len)
                        for i in eachindex(offsets)]
    residual = find_crossings(final_connectors)
    if !isempty(residual)
        @warn "repair_crossings! hit max_iter=$max_iter without convergence; $(length(residual)) crossing(s) remain"
    end
    return max_iter
end
