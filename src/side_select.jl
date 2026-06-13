# side_select.jl — greedy discrete Imhof-slot refinement. Pure, GeometryBasics-only.
#
# Each label's candidate offsets are its in-bounds Imhof slots (constrained by
# only_move). Seeded from the Voronoi-informed init, then refined by index-ordered
# greedy sweeps minimizing the lexicographic key
#   (hard_overlaps, leader_length)
# where hard_overlaps counts label–label box overlaps, label–obstacle overlaps, AND
# label–marker point overlaps (foreign anchors covered by the label box). Overlap
# avoidance provably dominates leader length. Pinned labels are fixed; obstacles are
# fixed boxes; a label never avoids its own anchor. Deterministic.

"""
    side_select(anchors, sizes, psizes, bounds, seed, params;
                pin_mask=nothing, pinned_offsets=Vec2f[], obstacles=Rect2f[],
                passes=6) -> Vector{Vec2f}

Pick, per label, the Imhof slot minimizing the lexicographic key
`(hard_overlaps, leader_length)`. `psizes` are padded sizes. `seed[i]` is the
Voronoi-informed initial offset used to choose the starting slot. Returns the
chosen offset per label.
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
    cands = Vector{Vector{Vec2f}}(undef, n)
    for i in 1:n
        cs = Vec2f[]
        for s in IMHOF_ORDER
            o = _constrain(slot_offset(s, sizes[i], p), params.only_move)
            inb(box_at(anchors[i], o, psizes[i])) && push!(cs, o)
        end
        isempty(cs) && (cs = [_constrain(slot_offset(s, sizes[i], p), params.only_move)
                              for s in IMHOF_ORDER])
        cands[i] = cs
    end

    isfixed(i) = pin_mask !== nothing && pin_mask[i]

    # initial selection: pinned → fixed offset; else nearest candidate to seed
    sel = Vector{Vec2f}(undef, n)
    for i in 1:n
        if isfixed(i)
            sel[i] = pinned_offsets[i]
        else
            best = cands[i][1]; bestd = Inf
            for o in cands[i]
                d = (Float64(o[1]) - seed[i][1])^2 + (Float64(o[2]) - seed[i][2])^2
                if d < bestd; bestd = d; best = o; end
            end
            sel[i] = best
        end
    end

    # Lexicographic arrangement key: (hard_overlaps, soft). hard_overlaps = label–label
    # overlap pairs + label–marker point overlaps (W_pt = W_lap: same lex level). soft =
    # total leader length (the tiebreak). Compared with Julia tuple `<` (lexicographic),
    # so overlap-avoidance provably dominates leader length regardless of pixel scale.
    # Greedy best-response is NOT globally monotone and can 2-cycle, so we keep the best
    # arrangement seen across passes rather than trusting the last pass. Obstacle overlaps
    # are counted once per (label, obstacle) pair, consistent with the greedy `ov` below.
    function global_key(s)
        hard = 0
        soft = 0.0
        for i in 1:n
            b   = box_at(anchors[i], s[i], psizes[i])          # box-padded, for label–label
            bm  = box_at(anchors[i], s[i], sizes[i])           # unpadded text box, for markers
            for j in (i+1):n
                (overlap_push(b, box_at(anchors[j], s[j], psizes[j])) != Vec2f(0, 0)) && (hard += 1)
            end
            for ob in obstacles
                (overlap_push(b, ob) != Vec2f(0, 0)) && (hard += 1)
            end
            for j in 1:n                                       # foreign markers (own anchor skipped)
                j == i && continue
                point_covered(anchors[j], bm, p) && (hard += 1)
            end
            soft += sqrt(Float64(s[i][1])^2 + Float64(s[i][2])^2)
        end
        return (hard, soft)
    end

    best_sel = copy(sel); best_key = global_key(sel)
    for _ in 1:passes
        changed = false
        for i in 1:n
            isfixed(i) && continue
            besto = sel[i]; bestkey = (typemax(Int), Inf)
            for o in cands[i]
                b  = box_at(anchors[i], o, psizes[i])
                bm = box_at(anchors[i], o, sizes[i])
                ov = 0
                for j in 1:n
                    j == i && continue
                    (overlap_push(b, box_at(anchors[j], sel[j], psizes[j])) != Vec2f(0, 0)) && (ov += 1)
                    point_covered(anchors[j], bm, p) && (ov += 1)   # foreign marker term
                end
                for ob in obstacles
                    (overlap_push(b, ob) != Vec2f(0, 0)) && (ov += 1)
                end
                soft = sqrt(Float64(o[1])^2 + Float64(o[2])^2)
                key = (ov, soft)
                if key < bestkey; bestkey = key; besto = o; end
            end
            (besto != sel[i]) && (changed = true)
            sel[i] = besto
        end
        gk = global_key(sel)
        if gk < best_key; best_key = gk; best_sel = copy(sel); end
        changed || break
    end
    return best_sel
end
