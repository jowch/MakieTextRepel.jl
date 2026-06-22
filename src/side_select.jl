# side_select.jl — greedy discrete Imhof-slot refinement. Pure, GeometryBasics-only.
#
# Candidates: in-bounds Imhof slots, constrained by only_move. Seeded from
# Voronoi-informed init, refined by index-ordered greedy sweeps minimizing
#   (hard_overlaps, leader_length, imhof_rank)
# hard_overlaps: label–label + label–obstacle + label–marker point overlaps
# (W_pt = W_lap: marker point overlaps count at the same level as box overlaps).
# rank: Imhof slot index TR=0…TL=7; breaks exact-leader ties toward upper/right,
# never lengthens a leader. Pinned labels fixed; obstacles fixed boxes; a label
# never avoids its own anchor. Deterministic.

"""
    side_select(anchors, sizes, psizes, bounds, seed, params;
                pin_mask=nothing, pinned_offsets=Vec2f[], obstacles=Rect2f[],
                passes=6) -> Vector{Vec2f}

Per label, pick the Imhof slot minimizing `(hard_overlaps, leader_length, imhof_rank)`.
`psizes` are padded sizes. `seed[i]` is the Voronoi-informed initial offset for slot
seeding. Returns one offset per label.
"""
function side_select(anchors::Vector{Point2f}, sizes::Vector{Vec2f},
                     psizes::Vector{Vec2f}, bounds::Rect2f,
                     seed::Vector{Vec2f}, params;
                     pin_mask::Union{Nothing,BitVector} = nothing,
                     pinned_offsets::Vector{Vec2f}       = Vec2f[],
                     obstacles::Vector{Rect2f}           = Rect2f[],
                     passes::Int = 6)
    n = length(anchors)
    if pin_mask !== nothing
        length(pin_mask) == n || throw(DimensionMismatch(
            "pin_mask length $(length(pin_mask)) does not match anchors length $n"))
        length(pinned_offsets) == n || throw(DimensionMismatch(
            "pinned_offsets length $(length(pinned_offsets)) does not match anchors length $n"))
    end
    p = params.point_padding
    blo = bounds.origin; bw = bounds.widths
    inb(b) = b.origin[1] >= blo[1] - 1e-3 && b.origin[2] >= blo[2] - 1e-3 &&
             b.origin[1] + b.widths[1] <= blo[1] + bw[1] + 1e-3 &&
             b.origin[2] + b.widths[2] <= blo[2] + bw[2] + 1e-3

    # candidate offsets per label: in-bounds, only_move-constrained Imhof slots
    # (keep all eight if none fit, so the legalizer can rescue it later)
    cands = Vector{Vector{Tuple{Vec2f,Int}}}(undef, n)
    for i in 1:n
        cs = Tuple{Vec2f,Int}[]
        for (rank, s) in enumerate(IMHOF_ORDER)
            o = _constrain(slot_offset(s, sizes[i], p), params.only_move)
            inb(box_at(anchors[i], o, psizes[i])) && push!(cs, (o, rank - 1))
        end
        isempty(cs) && (cs = [(_constrain(slot_offset(s, sizes[i], p), params.only_move), rank - 1)
                              for (rank, s) in enumerate(IMHOF_ORDER)])
        cands[i] = cs
    end

    isfixed(i) = pin_mask !== nothing && pin_mask[i]

    # initial selection: pinned → fixed offset; else nearest candidate to seed
    sel = Vector{Vec2f}(undef, n)
    sel_rank = zeros(Int, n)            # Imhof rank of each label's currently-selected slot
    for i in 1:n
        if isfixed(i)
            sel[i] = pinned_offsets[i]
        else
            best = cands[i][1][1]; bestrank = cands[i][1][2]; bestd = Inf
            for (o, rank) in cands[i]
                d = (Float64(o[1]) - seed[i][1])^2 + (Float64(o[2]) - seed[i][2])^2
                if d < bestd; bestd = d; best = o; bestrank = rank; end
            end
            sel[i] = best; sel_rank[i] = bestrank
        end
    end

    # Global arrangement key: (hard_overlaps, leader, ranksum).
    # hard_overlaps: box–box pairs + (label,obstacle) pairs + marker point overlaps
    # (W_pt = W_lap). leader: total leader length. ranksum: sum of Imhof ranks.
    # Compared via Julia tuple `<`. Greedy best-response can 2-cycle, so we keep
    # the best arrangement seen across passes. Takes explicit `s_rank` (not the
    # outer `sel_rank`) so the key is a pure function of its arguments.
    function global_key(s, s_rank)
        hard = 0
        leader = 0.0
        ranksum = 0
        for i in 1:n
            b   = box_at(anchors[i], s[i], psizes[i])
            bm  = box_at(anchors[i], s[i], sizes[i])
            for j in (i+1):n
                boxes_overlap(b, box_at(anchors[j], s[j], psizes[j])) && (hard += 1)
            end
            for ob in obstacles
                boxes_overlap(b, ob) && (hard += 1)
            end
            for j in 1:n
                j == i && continue
                point_covered(anchors[j], bm, p) && (hard += 1)
            end
            leader += sqrt(Float64(s[i][1])^2 + Float64(s[i][2])^2)
            ranksum += s_rank[i]
        end
        return (hard, leader, ranksum)
    end

    best_sel = copy(sel); best_key = global_key(sel, sel_rank)
    for _ in 1:passes
        changed = false
        for i in 1:n
            isfixed(i) && continue
            besto = sel[i]; bestrank = sel_rank[i]; bestkey = (typemax(Int), Inf, typemax(Int))
            for (o, rank) in cands[i]
                b  = box_at(anchors[i], o, psizes[i])
                bm = box_at(anchors[i], o, sizes[i])
                ov = 0
                for j in 1:n
                    j == i && continue
                    boxes_overlap(b, box_at(anchors[j], sel[j], psizes[j])) && (ov += 1)
                    point_covered(anchors[j], bm, p) && (ov += 1)
                end
                for ob in obstacles
                    boxes_overlap(b, ob) && (ov += 1)
                end
                key = (ov, sqrt(Float64(o[1])^2 + Float64(o[2])^2), rank)
                if key < bestkey; bestkey = key; besto = o; bestrank = rank; end
            end
            (besto != sel[i]) && (changed = true)
            sel[i] = besto; sel_rank[i] = bestrank
        end
        gk = global_key(sel, sel_rank)
        if gk < best_key; best_key = gk; best_sel = copy(sel); end
        changed || break
    end
    return best_sel
end
