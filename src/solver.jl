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
