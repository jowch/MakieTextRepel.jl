# crossings.jl — leader-segment crossing detection and 2-opt repair.

"""
Strict segment-segment intersection test via signed-area orientation.
Returns `true` only when the segments properly cross (each segment's
endpoints lie on opposite sides of the other line). Endpoint-touching
and collinear-overlap return `false`.
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
Exchange absolute positions of labels `i` and `j` while preserving their
(label_text → anchor) identity. The new offsets are computed so each label's
absolute position becomes the other's old absolute position.
"""
function swap_positions!(offsets::Vector{Vec2f}, anchors::Vector{Point2f}, i::Int, j::Int)
    pos_i_old = anchors[i] .+ offsets[i]
    pos_j_old = anchors[j] .+ offsets[j]
    offsets[i] = Vec2f(pos_j_old .- anchors[i])
    offsets[j] = Vec2f(pos_i_old .- anchors[j])
    return offsets
end

"""
Iterate: scan for crossings; for each non-conflicting crossing pair swap label
positions; repeat. Terminates when no crossings remain or `max_iter` exceeded.
`min_len` matches `min_segment_length` from the recipe so the scan agrees with
what `build_connectors` would render. No default — callers must pass the value
the renderer will use, so the scan and the render agree byte-for-byte.

Returns the number of outer iterations consumed (0 if no crossings on first scan).
On cap-out, performs one final rescan and emits a `@warn` listing the residual
crossings — the "best-effort with backstop" signal.
The non-crossing guarantee holds whenever this function returns < `max_iter`,
*except* for crossings that touch a pinned label (via `pin_mask`): those are never
swapped, so the early no-progress return can leave such pinned crossings in place.
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
        # All remaining crossings touch a pinned label → no swap is possible.
        # Return early instead of burning the rest of max_iter (and emitting the
        # spurious "without convergence" @warn over crossings we are intentionally
        # leaving in place). The residual is expected, not a failure.
        isempty(swapped) && return iter
    end
    # Final rescan — distinguishes "capped out and converged on the last swap"
    # from "capped out with crossings still present". Only warn for the latter.
    final_connectors = [connector_for(anchors[i], offsets[i], sizes[i], dropped[i], params, min_len)
                        for i in eachindex(offsets)]
    residual = find_crossings(final_connectors)
    if !isempty(residual)
        @warn "repair_crossings! hit max_iter=$max_iter without convergence; $(length(residual)) crossing(s) remain"
    end
    return max_iter
end
