# solvers/projection.jl — ProjectionSolver: side-select → repair → legalize, with
# geometric over-capacity dropping and read-only Q diagnostics. Composes the pure
# layers; the only AbstractClusterSolver that touches solver-internal types here.

"""
`ProjectionSolver` carries `RepelParams` and a `stats` Ref holding the last solve's
Q diagnostics `(overlaps, mean_leader, crossings, iter, residual, dropped)`.
"""
struct ProjectionSolver <: AbstractClusterSolver
    params::RepelParams
    stats::Base.RefValue{NamedTuple}
end

ProjectionSolver(params::RepelParams) =
    ProjectionSolver(params, Ref{NamedTuple}((; overlaps = 0, mean_leader = 0f0,
                                                crossings = 0, iter = 0,
                                                residual = 0f0, dropped = 0)))

"""
Mark the still-active, non-pinned label whose padded box overlaps the most other
active boxes (ties → highest index) as dropped. Returns the dropped index (0 if
none eligible). Deterministic.
"""
function drop_most_overlapped!(dropped::BitVector, anchors::Vector{Point2f},
                               offsets::Vector{Vec2f}, psizes::Vector{Vec2f},
                               pin_mask::Union{Nothing,BitVector})
    n = length(offsets)
    bestidx = 0; bestov = -1
    for i in 1:n
        dropped[i] && continue
        (pin_mask !== nothing && pin_mask[i]) && continue
        bi = box_at(anchors[i], offsets[i], psizes[i])
        ov = 0
        for j in 1:n
            (j == i || dropped[j]) && continue
            (overlap_push(bi, box_at(anchors[j], offsets[j], psizes[j])) != Vec2f(0, 0)) && (ov += 1)
        end
        if ov > bestov || (ov == bestov && i > bestidx)   # ties → highest index
            bestov = ov; bestidx = i
        end
    end
    bestidx > 0 && (dropped[bestidx] = true)
    return bestidx
end

function solve_cluster(s::ProjectionSolver, anchors::Vector{Point2f}, sizes::Vector{Vec2f},
                       bounds::Rect2f;
                       init_state::Union{Nothing,Vector{Vec2f}} = nothing,
                       pin_mask::Union{Nothing,BitVector}        = nothing,
                       pinned_offsets::Vector{Vec2f}             = Vec2f[],
                       obstacles::Vector{Rect2f}                 = Rect2f[])
    p = RepelParams(s.params; bounds = bounds)
    n = length(anchors)

    # Mirror solve_repel's input validation (src/solver.jl). Matters most on the
    # WARM path, where initial_offsets isn't called to catch a mismatch for us.
    if pin_mask !== nothing
        length(pin_mask) == n || throw(DimensionMismatch(
            "pin_mask length $(length(pin_mask)) does not match anchors length $n"))
        length(pinned_offsets) == n || throw(DimensionMismatch(
            "pinned_offsets length $(length(pinned_offsets)) does not match anchors length $n"))
    end
    if init_state !== nothing
        length(init_state) == n || throw(DimensionMismatch(
            "init_state length $(length(init_state)) does not match anchors length $n"))
    end

    pad = Float32(p.box_padding)
    psizes = [sizes[i] .+ 2 * pad for i in 1:n]

    if init_state === nothing       # FRESH: seed → side-select → crossing repair
        seed = initial_offsets(anchors, sizes, voronoi_cells(anchors, bounds), p;
                               pin_mask = pin_mask, pinned_offsets = pinned_offsets)
        offsets = side_select(anchors, sizes, psizes, bounds, seed, p;
                              pin_mask = pin_mask, pinned_offsets = pinned_offsets,
                              obstacles = obstacles)
        repair_crossings!(offsets, anchors, sizes, falses(n), p;
                          min_len = p.min_segment_length, pin_mask = pin_mask)
    else                            # RELAX / warm-start: legalize the given layout only.
        # Mirror solve_repel's init_state contract: constrain to only_move, and hold
        # pinned labels at their fixed offset (the caller's pinned_offsets), not the
        # warm value. Without this, pinned labels would be legalized away from their pin.
        offsets = [_constrain(o, p.only_move) for o in init_state]
        if pin_mask !== nothing
            for i in 1:n
                pin_mask[i] && (offsets[i] = pinned_offsets[i])
            end
        end
    end

    dropped = falses(n)
    local lz = (; offsets = offsets, residual = 0f0, rounds_used = 0)
    while true
        # working arrays = active (non-dropped) labels ∪ obstacle pseudo-nodes (all fixed)
        act = Int[i for i in 1:n if !dropped[i]]
        m = length(act); k = length(obstacles)
        w_anchors = Vector{Point2f}(undef, m + k)
        w_offsets = Vector{Vec2f}(undef, m + k)
        w_psizes  = Vector{Vec2f}(undef, m + k)
        w_fixed   = falses(m + k)
        for (t, i) in enumerate(act)
            w_anchors[t] = anchors[i]; w_offsets[t] = offsets[i]; w_psizes[t] = psizes[i]
            (pin_mask !== nothing && pin_mask[i]) && (w_fixed[t] = true)
        end
        for (t, ob) in enumerate(obstacles)
            w_anchors[m + t] = Point2f(ob.origin .+ ob.widths ./ 2)
            w_offsets[m + t] = Vec2f(0, 0)
            w_psizes[m + t]  = Vec2f(ob.widths)
            w_fixed[m + t]   = true
        end
        lz = legalize(w_anchors, w_offsets, w_psizes, bounds;
                      fixed = w_fixed, only_move = p.only_move)
        # Only active labels are written back; dropped labels keep their last pre-drop
        # position (filtered by `dropped` everywhere downstream).
        for (t, i) in enumerate(act)
            offsets[i] = lz.offsets[t]
        end
        (lz.residual ≤ 0.5f0 || count(!, dropped) ≤ 1) && break
        idx = drop_most_overlapped!(dropped, anchors, offsets, psizes, pin_mask)
        # idx == 0: no label is *eligible* to drop (all remaining survivors pinned) —
        # distinct from the one-survivor case already caught by `count(!, dropped) ≤ 1`
        # in the break condition above. Stop here; the residual warn below still fires.
        idx == 0 && break
    end

    if lz.residual > 0.5f0
        @warn "ProjectionSolver: residual overlap after dropping; scene over-capacity for bounds=$bounds"
    end

    q = label_cost(anchors, sizes; offsets = offsets, bounds = bounds, dropped = dropped,
                   box_padding = p.box_padding, point_padding = p.point_padding,
                   min_segment_length = p.min_segment_length)
    s.stats[] = (; overlaps = q.overlaps, mean_leader = q.mean_leader, crossings = q.crossings,
                   iter = lz.rounds_used, residual = lz.residual, dropped = count(dropped))
    return (; offsets = offsets, dropped = dropped, iter = lz.rounds_used, residual = lz.residual)
end
