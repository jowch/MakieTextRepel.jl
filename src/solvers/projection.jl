# solvers/projection.jl — ProjectionSolver: side-select → repair → legalize, geometric
# drop loop, read-only Q diagnostics. Composes the pure layers.

# Concrete NamedTuple alias keeps the stats Ref type-stable and enforces the same
# field set/order at every write site (constructor, solve_cluster, all-pinned bypass).
const ProjectionStats = NamedTuple{(:overlaps, :point_overlaps, :mean_leader, :crossings, :iter, :residual, :dropped),
                                   Tuple{Int, Int, Float32, Int, Int, Float32, Int}}

"""
`ProjectionSolver` carries `RepelParams` and a `stats` Ref for the last solve's
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
Drop the still-active, non-pinned label whose padded box overlaps the most other
active boxes and obstacles (ties → highest index). Obstacle overlaps are counted so a
label-vs-obstacle residual doesn't score `ov = 0` and cause a wrong fallback drop.
Returns the dropped index (0 if none eligible).
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
            boxes_overlap(bi, box_at(anchors[j], offsets[j], psizes[j])) && (ov += 1)
        end
        for ob in obstacles
            boxes_overlap(bi, ob) && (ov += 1)
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

    # Input validation — mirrors solve_repel (force_model.jl). Critical on the warm
    # path, where initial_offsets isn't called to catch a mismatch first.
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
    else                            # WARM: legalize the given layout only.
        # Constrain axes and restore pinned offsets (don't legalize pinned labels away).
        offsets = [_constrain(o, p.only_move) for o in init_state]
        if pin_mask !== nothing
            for i in 1:n
                pin_mask[i] && (offsets[i] = pinned_offsets[i])
            end
        end
    end

    # Marker keep-out half-extent. Label half-extent = unpadded_half + box_padding.
    # A fixed keep-out of half-extent mc separates to unpadded_half + point_padding,
    # so the text edge clears the marker by exactly point_padding (matches point_covered).
    mc = Float32(p.point_padding - p.box_padding)

    # Legalize → drop loop to convergence on a given offsets vector.
    # Returns (offsets, dropped, lz). Mutates only locals.
    function legalize_and_drop(start_offsets::Vector{Vec2f})
        offs = copy(start_offsets)
        drp  = falses(n)
        local lz = (; offsets = offs, residual = 0f0, rounds_used = 0)
        while true
            act = Int[i for i in 1:n if !drp[i]]
            m = length(act); k = length(obstacles)
            tot = m + k + n
            w_anchors = Vector{Point2f}(undef, tot)
            w_offsets = Vector{Vec2f}(undef, tot)
            w_psizes  = Vector{Vec2f}(undef, tot)
            w_fixed   = falses(tot)
            w_soft    = falses(tot)
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
            # Marker keep-outs: all anchors (own + foreign, incl. dropped/pinned), fixed +
            # soft, ascending index → deterministic Dykstra order, stable across rounds.
            for i in 1:n
                w_anchors[m + k + i] = anchors[i]
                w_offsets[m + k + i] = Vec2f(0, 0)
                w_psizes[m + k + i]  = Vec2f(2mc, 2mc)   # psize = full width; mc is the half-extent
                w_fixed[m + k + i]   = true
                w_soft[m + k + i]    = true
            end
            lz = legalize(w_anchors, w_offsets, w_psizes, bounds;
                          fixed = w_fixed, soft = w_soft, only_move = p.only_move)
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
    total_rounds = lz.rounds_used   # accumulates across swap rounds; iter reflects whole solve.

    # Swap-based crossing elimination. Scan crossing pairs; adopt the first swap whose
    # post-legalize swapkey = (dropped_count, overlaps+point_overlaps, crossings, mean_leader)
    # strictly improves; restart. Terminates: each accepted swap strictly decreases swapkey
    # over the finite swap-reachable set (no RNG; index-ordered). Residual crossings (a
    # 2-opt-resistant 3-cycle or true conflict) escape with a @warn; overlaps always outrank
    # crossings — zero-overlap is never traded for fewer crossings; dropped_count outranks all.
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
