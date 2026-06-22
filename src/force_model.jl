# force_model.jl — pure deterministic force-directed label repel (no Makie types).
#
# This is the NON-DEFAULT placement path: the in-tree fallback behind the
# `AbstractClusterSolver` seam (wrapped by solvers/force.jl as `ForceSolver`).
# The default solver is `ProjectionSolver` (solvers/projection.jl). `RepelParams`
# and the shared `_constrain` axis-lock helper live in params.jl.
# `_GOLDEN_ANGLE` is also defined in params.jl (shared by the default-path init).

"""
Deterministic initial offsets. Each label gets a golden-angle direction sized
to its padded box corner-distance, placing the anchor on or outside the box.
`psizes` is padded size (caller adds `2·box_padding`). The `1.0f0` floor
binds only for zero-size labels with zero padding.
"""
function init_offsets(anchors::Vector{Point2f}, psizes::Vector{Vec2f}, p::RepelParams)
    n = length(anchors)
    offsets = Vector{Vec2f}(undef, n)
    for i in 1:n
        hw = psizes[i][1] / 2
        hh = psizes[i][2] / 2
        r  = max(sqrt(hw*hw + hh*hh), 1f0)
        θ  = _GOLDEN_ANGLE * Float32(i)
        offsets[i] = Vec2f(r * cos(θ), r * sin(θ))
    end
    return offsets
end

_clamp_step(d::Vec2f, m::Float32) = (n = norm(d); (n == 0 || n <= m) ? d : d .* (m / n))

"""
Mark labels whose final box overlaps more than `max_overlaps` other label boxes.
`Inf` keeps everything.
"""
function compute_drops(anchors::Vector{Point2f}, offsets::Vector{Vec2f},
                       psizes::Vector{Vec2f}, max_overlaps::Real)
    n = length(anchors)
    dropped = falses(n)
    isinf(max_overlaps) && return dropped
    boxes = [box_at(anchors[i], offsets[i], psizes[i]) for i in 1:n]
    for i in 1:n
        count = 0
        for j in 1:n
            i == j && continue
            boxes_overlap(boxes[i], boxes[j]) && (count += 1)
        end
        dropped[i] = count > max_overlaps
    end
    return dropped
end

"""
Solve label offsets (pixels) so boxes avoid each other and their anchor points.

Returns `(offsets::Vector{Vec2f}, dropped::BitVector)`. `anchors` and `sizes`
are in pixels; a label's box is centered at `anchor + offset`, padded by
`params.box_padding`.
"""
function solve_repel(anchors::Vector{Point2f}, sizes::Vector{Vec2f}, p::RepelParams;
                     obstacles::Vector{Rect2f}                = Rect2f[],
                     init_state::Union{Nothing,Vector{Vec2f}} = nothing,
                     pin_mask::Union{Nothing,BitVector}        = nothing,
                     pinned_offsets::Vector{Vec2f}             = Vec2f[])
    n = length(anchors)
    n == 0 && return (; offsets = Vec2f[], dropped = falses(0), iter = 0, residual = 0f0)
    @assert length(sizes) == n "anchors and sizes must have equal length"

    psizes = [s .+ 2 * Float32(p.box_padding) for s in sizes]
    if init_state !== nothing
        length(init_state) == n || throw(DimensionMismatch(
            "init_state length $(length(init_state)) does not match anchors length $n"))
        offsets = [_constrain(o, p.only_move) for o in init_state]
    else
        offsets = [_constrain(o, p.only_move) for o in init_offsets(anchors, psizes, p)]
    end

    if pin_mask !== nothing
        length(pin_mask) == n || throw(DimensionMismatch(
            "pin_mask length $(length(pin_mask)) does not match anchors length $n"))
        length(pinned_offsets) == n || throw(DimensionMismatch(
            "pinned_offsets length $(length(pinned_offsets)) does not match anchors length $n"))
        for i in 1:n
            if pin_mask[i]
                offsets[i] = pinned_offsets[i]   # bypasses only_move (D6)
            end
        end
    end

    fx, fy   = Float32.(p.force)
    ppx, ppy = Float32.(p.force_point)
    plx, ply = Float32.(p.force_pull)
    pad      = Float32(p.point_padding)
    smax0    = Float32(p.step_max)
    pthr     = Float32(p.pull_threshold)

    final_iter = 0
    final_residual = 0f0

    for it in 1:p.max_iter
        # Step-cap cooling: linear decay of the per-iteration move cap.
        # DETERMINISM: `bounds === nothing` path is byte-identical to pre-clamping output;
        # cooling and clamping only run when bounds is set. Preserve this when editing.
        smax = p.bounds === nothing ? smax0 :
               smax0 * max(0f0, 1f0 - Float32(it) / Float32(p.max_iter))
        boxes = [box_at(anchors[i], offsets[i], psizes[i]) for i in 1:n]
        Δ = Vector{Vec2f}(undef, n)
        for i in 1:n
            if pin_mask !== nothing && pin_mask[i]
                Δ[i] = Vec2f(0, 0)
                continue
            end
            f = Vec2f(0, 0)
            for j in 1:n
                i == j && continue
                push = overlap_push(boxes[i], boxes[j])
                f = f .+ Vec2f(push[1] * fx, push[2] * fy)
            end
            for j in 1:n
                # Own anchor included; force_pull (below) provides inward balance.
                pp = point_push(boxes[i], anchors[j], pad)
                f = f .+ Vec2f(pp[1] * ppx, pp[2] * ppy)
            end
            for ob in obstacles
                push = overlap_push(boxes[i], ob)
                f = f .+ Vec2f(push[1] * fx, push[2] * fy)
            end
            off = offsets[i]
            if norm(off) > pthr
                f = f .- Vec2f(off[1] * plx, off[2] * ply)
            end
            Δ[i] = f
        end
        maxmove = 0f0
        for i in 1:n
            if pin_mask !== nothing && pin_mask[i]
                continue  # pinned: skip update, keep pinned_offsets[i]
            end
            d = _constrain(_clamp_step(Δ[i], smax), p.only_move)
            newoff = offsets[i] .+ d
            if p.bounds !== nothing
                box = box_at(anchors[i], newoff, psizes[i])
                # Constrain the clamp shift too, so confinement never moves
                # a label along an axis the user locked via only_move.
                newoff = newoff .+ _constrain(clamp_box_offset(box, p.bounds), p.only_move)
            end
            move = newoff .- offsets[i]
            offsets[i] = newoff
            maxmove = max(maxmove, norm(move))
        end
        final_iter = it
        final_residual = maxmove
        maxmove < p.tol && break
    end

    return (;
        offsets,
        dropped = compute_drops(anchors, offsets, psizes, p.max_overlaps),
        iter    = final_iter,
        residual = final_residual,
    )
end
