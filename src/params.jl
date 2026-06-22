# params.jl — shared solver configuration. Pure, GeometryBasics-only.
# Consumed by both ProjectionSolver (solvers/projection.jl) and
# ForceSolver/solve_repel (force_model.jl); lives above the solver seam.

# Golden angle (radians). Used by init.jl (coincident-anchor fan-out) and
# force_model.jl (initial-offset spiral).
const _GOLDEN_ANGLE = Float32(π * (3 - sqrt(5)))

"""Solver parameters. All distances in pixels."""
Base.@kwdef struct RepelParams
    force::NTuple{2,Float64}        = (1.0, 1.0)
    force_point::NTuple{2,Float64}  = (1.0, 1.0)
    force_pull::NTuple{2,Float64}   = (0.01, 0.01)
    max_iter::Int                   = 2000
    only_move::Symbol               = :both        # :both | :x | :y
    box_padding::Float64            = 4.0
    point_padding::Float64          = 0.0   # px; gap from each anchor/marker to the nearest label text edge (ProjectionSolver: soft keep-out; ForceSolver: point-repulsion halo). Primitive default 0.0; user surfaces (recipe, TextRepelAlgorithm) default to 5.0.
    max_overlaps::Float64           = Inf
    step_max::Float64               = 10.0          # per-iteration px clamp
    pull_threshold::Float64         = 1.0           # px; suppress spring within this
    tol::Float64                    = 0.1           # convergence: max move < tol
    min_segment_length::Float64     = 2.0           # px; min visible leader length (connector filter + crossing-repair threshold)
    bounds::Union{Rect2f, Nothing} = nothing   # clamp region in solver (pixel) space; nothing = no clamp
end

"""
    RepelParams(base::RepelParams; kwargs...) -> RepelParams

Copy `base`, replacing any fields named in `kwargs`. All `RepelParams`
fields not in `kwargs` are carried over unchanged.
"""
function RepelParams(base::RepelParams; kwargs...)
    return RepelParams(;
        (field => get(kwargs, field, getfield(base, field))
         for field in fieldnames(RepelParams))...)
end

# Apply the `only_move` axis lock to a displacement.
_constrain(d::Vec2f, mode::Symbol) =
    mode === :x ? Vec2f(d[1], 0) : mode === :y ? Vec2f(0, d[2]) : d
