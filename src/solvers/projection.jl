# solvers/projection.jl — ProjectionSolver: side-select → repair → legalize, with
# geometric over-capacity dropping and read-only Q diagnostics. Composes the pure
# layers; the only AbstractClusterSolver that touches solver-internal types here.

# Concrete shape of the Q-diagnostics tuple. Using a concrete alias (rather than an
# abstract `NamedTuple`) keeps the `stats` Ref type-stable and enforces the same field
# set/types at every write site: the constructor below, `solve_cluster`, and the
# annotation all-pinned bypass. Field order matches those write sites (no conversion).
const ProjectionStats = NamedTuple{(:overlaps, :point_overlaps, :mean_leader, :crossings, :iter, :residual, :dropped),
                                   Tuple{Int, Int, Float32, Int, Int, Float32, Int}}

"""
`ProjectionSolver` carries `RepelParams` and a `stats` Ref holding the last solve's
Q diagnostics `(overlaps, point_overlaps, mean_leader, crossings, iter, residual, dropped)`.
"""
struct ProjectionSolver <: AbstractClusterSolver
    params::RepelParams
    stats::Base.RefValue{ProjectionStats}
end

ProjectionSolver(params::RepelParams) =
    ProjectionSolver(params, Ref{ProjectionStats}((; overlaps = 0, point_overlaps = 0,
                                                     mean_leader = 0f0, crossings = 0,
                                                     iter = 0, residual = 0f0, dropped = 0)))

"""
Mark the still-active, non-pinned label whose padded box overlaps the most other
active boxes **and obstacles** (ties → highest index) as dropped. Counting obstacle
overlaps matters when a round's residual comes purely from a label-vs-obstacle
penetration: without it, such a label scores `ov = 0` and the highest-index `ov = 0`
fallback could drop the wrong label. Returns the dropped index (0 if none eligible).
Deterministic.
"""
function drop_most_overlapped!(dropped::BitVector, anchors::Vector{Point2f},
                               offsets::Vector{Vec2f}, psizes::Vector{Vec2f},
                               pin_mask::Union{Nothing,BitVector},
                               obstacles::Vector{Rect2f} = Rect2f[])
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
        for ob in obstacles
            (overlap_push(bi, ob) != Vec2f(0, 0)) && (ov += 1)
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

    # Mirror solve_repel's input validation (src/force_model.jl). Matters most on the
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

    # Run the legalize → over-capacity-drop loop to convergence on a *given* offsets vector.
    # Returns (offsets, dropped, lz). Pure w.r.t. its inputs (mutates only locals).
    function legalize_and_drop(start_offsets::Vector{Vec2f})
        offs = copy(start_offsets)
        drp  = falses(n)
        local lz = (; offsets = offs, residual = 0f0, rounds_used = 0)
        while true
            act = Int[i for i in 1:n if !drp[i]]
            m = length(act); k = length(obstacles)
            w_anchors = Vector{Point2f}(undef, m + k)
            w_offsets = Vector{Vec2f}(undef, m + k)
            w_psizes  = Vector{Vec2f}(undef, m + k)
            w_fixed   = falses(m + k)
            for (t, i) in enumerate(act)
                w_anchors[t] = anchors[i]; w_offsets[t] = offs[i]; w_psizes[t] = psizes[i]
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
            for (t, i) in enumerate(act)
                offs[i] = lz.offsets[t]
            end
            (lz.residual ≤ 0.5f0 || count(!, drp) ≤ 1) && break
            idx = drop_most_overlapped!(drp, anchors, offs, psizes, pin_mask, obstacles)
            idx == 0 && break
        end
        return (offs, drp, lz)
    end

    offsets, dropped, lz = legalize_and_drop(offsets)
    total_rounds = lz.rounds_used   # accumulates legalize rounds across the swap search too,
                                    # so reported `iter` reflects the whole solve, not just the
                                    # final accepted swap's legalize pass.

    # Stage 3 Part B: swap-based local search to drive crossings to zero. Scan all crossing
    # pairs for the first swap whose post-legalize key strictly improves, adopt it, restart.
    # swapkey = (dropped_count, overlaps+point_overlaps, crossings, mean_leader): the top
    # dropped_count level forbids killing a crossing by dropping a label; overlaps dominate
    # crossings dominate leader. Each adopted swap strictly decreases swapkey over the finite
    # swap-reachable layout set ⇒ terminates (crossing-free or local fixpoint) within the cap.
    # Deterministic (index-ordered, no RNG). Residual crossings (conflict, or a 2-opt-resistant
    # 3-cycle) are an honest escape hatch reported via solve_stats + the @warn below.
    UNCROSS_ROUNDS = 50
    swapkey(offs, drp) = let q = label_cost(anchors, sizes; offsets = offs, bounds = bounds,
                                            dropped = drp, box_padding = p.box_padding,
                                            point_padding = p.point_padding,
                                            min_segment_length = p.min_segment_length)
        (count(drp), q.overlaps + q.point_overlaps, q.crossings, q.mean_leader)
    end
    for _ in 1:UNCROSS_ROUNDS
        conns = [connector_for(anchors[i], offsets[i], sizes[i], dropped[i], p, p.min_segment_length)
                 for i in 1:n]
        X = find_crossings(conns)
        isempty(X) && break
        curkey = swapkey(offsets, dropped)
        improved = false
        for (i, j) in X
            (pin_mask !== nothing && (pin_mask[i] || pin_mask[j])) && continue
            trial = copy(offsets)
            swap_positions!(trial, anchors, i, j)
            toffs, tdrp, tlz = legalize_and_drop(trial)
            if swapkey(toffs, tdrp) < curkey
                offsets, dropped, lz = toffs, tdrp, tlz
                total_rounds += tlz.rounds_used
                improved = true
                break
            end
        end
        improved || break
    end

    if lz.residual > 0.5f0
        @warn "ProjectionSolver: residual overlap after dropping; scene over-capacity for bounds=$bounds"
    end

    q = label_cost(anchors, sizes; offsets = offsets, bounds = bounds, dropped = dropped,
                   box_padding = p.box_padding, point_padding = p.point_padding,
                   min_segment_length = p.min_segment_length)
    s.stats[] = (; overlaps = q.overlaps, point_overlaps = q.point_overlaps,
                   mean_leader = q.mean_leader, crossings = q.crossings,
                   iter = total_rounds, residual = lz.residual, dropped = count(dropped))
    return (; offsets = offsets, dropped = dropped, iter = total_rounds, residual = lz.residual)
end
