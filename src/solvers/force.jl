# solvers/force.jl — Force-directed cluster solver wrapping solve_repel.

"""ForceSolver carries `RepelParams` and dispatches to `solve_repel`."""
struct ForceSolver <: AbstractClusterSolver
    params::RepelParams
end

function solve_cluster(s::ForceSolver, anchors::Vector{Point2f}, sizes::Vector{Vec2f},
                       initial_offsets::Vector{Vec2f}, bounds::Rect2f)
    # `RepelParams(base; ...)` (src/solver.jl:25-29) copies `s.params` and overrides
    # only `bounds`, so callers don't need to mutate anything.
    p = RepelParams(s.params; bounds = bounds)
    r = solve_repel(anchors, sizes, p; init_state = initial_offsets)
    return (r.offsets, r.dropped)
end
