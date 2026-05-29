# side_select.jl — greedy discrete Imhof-slot refinement. Pure, GeometryBasics-only.
#
# Each label's candidate offsets are its in-bounds Imhof slots (constrained by
# only_move). Seeded from the Voronoi-informed init (nearest candidate to the
# seed), then refined by index-ordered greedy sweeps minimizing
#   cost(slot) = overlap_count·overlap_weight + ‖offset‖
# (overlap-avoidance dominates; leader length is the tiebreak). Pinned labels are
# fixed at their pinned offset; obstacles count as fixed boxes. Deterministic.

"""
    side_select(anchors, sizes, psizes, bounds, seed, params;
                pin_mask=nothing, pinned_offsets=Vec2f[], obstacles=Rect2f[],
                overlap_weight=1000.0, passes=6) -> Vector{Vec2f}

Pick, per label, the Imhof slot minimizing overlap then leader length. `psizes`
are padded sizes. `seed[i]` is the Voronoi-informed initial offset used to choose
the starting slot. Returns the chosen offset per label.
"""
function side_select(anchors::Vector{Point2f}, sizes::Vector{Vec2f},
                     psizes::Vector{Vec2f}, bounds::Rect2f,
                     seed::Vector{Vec2f}, params;
                     pin_mask::Union{Nothing,BitVector} = nothing,
                     pinned_offsets::Vector{Vec2f}       = Vec2f[],
                     obstacles::Vector{Rect2f}           = Rect2f[],
                     overlap_weight::Float64 = 1000.0, passes::Int = 6)
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

    # Global cost of an arrangement (overlap pairs · weight + total leader length).
    # Greedy best-response is NOT globally monotone and can 2-cycle, so we keep the
    # best arrangement seen across passes rather than trusting the last pass.
    # Obstacle overlaps are penalized once per (label, obstacle) pair — a per-label
    # penalty consistent with the greedy `ov` count below, so an obstacle overlapping
    # many labels pushes all of them off it.
    function global_cost(s)
        tot = 0.0
        for i in 1:n
            b = box_at(anchors[i], s[i], psizes[i])
            for j in (i+1):n
                (overlap_push(b, box_at(anchors[j], s[j], psizes[j])) != Vec2f(0, 0)) && (tot += overlap_weight)
            end
            for ob in obstacles
                (overlap_push(b, ob) != Vec2f(0, 0)) && (tot += overlap_weight)
            end
            tot += sqrt(Float64(s[i][1])^2 + Float64(s[i][2])^2)
        end
        return tot
    end

    best_sel = copy(sel); best_cost = global_cost(sel)
    for _ in 1:passes
        changed = false
        for i in 1:n
            isfixed(i) && continue
            besto = sel[i]; bestc = Inf
            for o in cands[i]
                b = box_at(anchors[i], o, psizes[i])
                ov = 0
                for j in 1:n
                    j == i && continue
                    (overlap_push(b, box_at(anchors[j], sel[j], psizes[j])) != Vec2f(0, 0)) && (ov += 1)
                end
                for ob in obstacles
                    (overlap_push(b, ob) != Vec2f(0, 0)) && (ov += 1)
                end
                c = ov * overlap_weight + sqrt(Float64(o[1])^2 + Float64(o[2])^2)
                if c < bestc; bestc = c; besto = o; end
            end
            (besto != sel[i]) && (changed = true)
            sel[i] = besto
        end
        gc = global_cost(sel)
        if gc < best_cost; best_cost = gc; best_sel = copy(sel); end
        changed || break
    end
    return best_sel
end
