# solver.jl — pure deterministic force-directed label repel (no Makie types).

"""Solver parameters. All distances in pixels."""
Base.@kwdef struct RepelParams
    force::NTuple{2,Float64}        = (1.0, 1.0)
    force_point::NTuple{2,Float64}  = (1.0, 1.0)
    force_pull::NTuple{2,Float64}   = (0.01, 0.01)
    max_iter::Int                   = 2000
    only_move::Symbol               = :both        # :both | :x | :y
    box_padding::Float64            = 4.0
    point_padding::Float64          = 0.0
    max_overlaps::Float64           = Inf
    step_max::Float64               = 10.0          # per-iteration px clamp
    pull_threshold::Float64         = 1.0           # px; suppress spring within this
    tol::Float64                    = 0.1           # convergence: max move < tol
    bounds::Union{Rect2f, Nothing} = nothing   # clamp region in solver (pixel) space; nothing = no clamp
end

const _GOLDEN_ANGLE = Float32(π * (3 - sqrt(5)))

"""
Deterministic initial offsets. Labels whose anchor coincides with an earlier
anchor are fanned out along a golden-angle spiral so the force loop has a
non-zero gradient to act on (replaces upstream random jitter).
"""
function explode_init(anchors::Vector{Point2f}, sizes::Vector{Vec2f}, p::RepelParams)
    n = length(anchors)
    offsets = fill(Vec2f(0, 0), n)
    # NOTE: nested loops (not `for i in 1:n, j in ...`) so `break` exits only the
    # inner j-loop; a fused loop's `break` would skip all remaining i.
    for i in 1:n
        for j in 1:(i - 1)
            if norm(anchors[i] .- anchors[j]) < 1f-3
                θ = _GOLDEN_ANGLE * i
                r = (sizes[i][1] + sizes[i][2]) / 4
                offsets[i] = offsets[i] .+ Vec2f(r * cos(θ), r * sin(θ))
                break
            end
        end
    end
    return offsets
end

_clamp_step(d::Vec2f, m::Float32) = (n = norm(d); (n == 0 || n <= m) ? d : d .* (m / n))

_constrain(d::Vec2f, mode::Symbol) =
    mode === :x ? Vec2f(d[1], 0) : mode === :y ? Vec2f(0, d[2]) : d

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
            overlap_push(boxes[i], boxes[j]) != Vec2f(0, 0) && (count += 1)
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
function solve_repel(anchors::Vector{Point2f}, sizes::Vector{Vec2f}, p::RepelParams)
    n = length(anchors)
    n == 0 && return (Vec2f[], falses(0))
    @assert length(sizes) == n "anchors and sizes must have equal length"

    psizes = [s .+ 2 * Float32(p.box_padding) for s in sizes]
    offsets = explode_init(anchors, psizes, p)

    fx, fy   = Float32.(p.force)
    ppx, ppy = Float32.(p.force_point)
    plx, ply = Float32.(p.force_pull)
    pad      = Float32(p.point_padding)
    smax     = Float32(p.step_max)
    pthr     = Float32(p.pull_threshold)

    for _ in 1:p.max_iter
        boxes = [box_at(anchors[i], offsets[i], psizes[i]) for i in 1:n]
        Δ = Vector{Vec2f}(undef, n)
        for i in 1:n
            f = Vec2f(0, 0)
            for j in 1:n
                i == j && continue
                push = overlap_push(boxes[i], boxes[j])
                f = f .+ Vec2f(push[1] * fx, push[2] * fy)
            end
            for j in 1:n
                i == j && continue   # don't repel a label from its OWN anchor
                pp = point_push(boxes[i], anchors[j], pad)
                f = f .+ Vec2f(pp[1] * ppx, pp[2] * ppy)
            end
            off = offsets[i]
            if norm(off) > pthr
                f = f .- Vec2f(off[1] * plx, off[2] * ply)
            end
            Δ[i] = f
        end
        maxmove = 0f0
        for i in 1:n
            d = _constrain(_clamp_step(Δ[i], smax), p.only_move)
            newoff = offsets[i] .+ d
            if p.bounds !== nothing
                box = box_at(anchors[i], newoff, psizes[i])
                newoff = newoff .+ clamp_box_offset(box, p.bounds)
            end
            move = newoff .- offsets[i]
            offsets[i] = newoff
            maxmove = max(maxmove, norm(move))
        end
        maxmove < p.tol && break
    end

    return (offsets, compute_drops(anchors, offsets, psizes, p.max_overlaps))
end
