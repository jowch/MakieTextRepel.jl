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
Deterministic initial offsets. Every label gets a per-index golden-angle
direction sized to escape its own (already padded) box, so the force loop
starts with each anchor on or outside its label box. Subsumes the old
"only fan out coincident anchors" behavior. Determinism: pure function of
index and passed-in sizes.

`psizes` is the *padded* size (the caller adds `2·box_padding`); `r` is the
corner-distance of the padded box. The anchor lands *on* the box only at the
four corner directions; at cardinal directions it lands `r − hw` (or `r − hh`)
*outside* the box, so equilibrium distance after spring relaxation varies a few
pixels by index. Acceptable for ggrepel-style layouts. The `1.0f0` floor only
binds for degenerate zero-size labels with zero padding (e.g. empty strings);
in normal layouts it never fires.
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
    n == 0 && return (; offsets = Vec2f[], dropped = falses(0), iter = 0, residual = 0f0)
    @assert length(sizes) == n "anchors and sizes must have equal length"

    psizes = [s .+ 2 * Float32(p.box_padding) for s in sizes]
    offsets = [_constrain(o, p.only_move) for o in init_offsets(anchors, psizes, p)]

    fx, fy   = Float32.(p.force)
    ppx, ppy = Float32.(p.force_point)
    plx, ply = Float32.(p.force_pull)
    pad      = Float32(p.point_padding)
    smax0    = Float32(p.step_max)
    pthr     = Float32(p.pull_threshold)

    final_iter = 0
    final_residual = 0f0

    for it in 1:p.max_iter
        # Step-cap cooling: linearly decay the per-iteration move cap so crowded,
        # wall-pinned labels settle instead of limit-cycling. Deterministic. Applied
        # only on the clamped path — the recipe always sets bounds, while the bare
        # `bounds === nothing` solver path stays byte-identical to its pre-clamping output.
        smax = p.bounds === nothing ? smax0 :
               smax0 * max(0f0, 1f0 - Float32(it) / Float32(p.max_iter))
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
                # Own anchor is included: keeps isolated labels off their own
                # point. `force_pull` (below) provides the inward balance, so
                # the label settles near (not on) the anchor, at `≈ hw + pad +
                # point_padding` along the dominant axis.
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
                # Constrain the clamp shift too, so confinement never moves a label
                # along an axis the user locked via only_move.
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
