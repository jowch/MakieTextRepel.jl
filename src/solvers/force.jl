# solvers/force.jl — Force-directed cluster solver wrapping solve_repel.

"""ForceSolver carries `RepelParams` and dispatches to `solve_repel`."""
struct ForceSolver <: AbstractClusterSolver
    params::RepelParams
end

function solve_cluster(s::ForceSolver, anchors::Vector{Point2f}, sizes::Vector{Vec2f},
                       bounds::Rect2f;
                       init_state::Union{Nothing,Vector{Vec2f}} = nothing,
                       pin_mask::Union{Nothing,BitVector}        = nothing,
                       pinned_offsets::Vector{Vec2f}             = Vec2f[],
                       obstacles::Vector{Rect2f}                 = Rect2f[])
    # `RepelParams(base; ...)` (src/params.jl) copies s.params, overriding bounds.
    p = RepelParams(s.params; bounds = bounds)
    fresh = init_state === nothing
    init = fresh ?
        initial_offsets(anchors, sizes, voronoi_cells(anchors, bounds), p;
                        pin_mask = pin_mask, pinned_offsets = pinned_offsets) :
        init_state
    r = solve_repel(anchors, sizes, p;
                    init_state = init, obstacles = obstacles,
                    pin_mask = pin_mask, pinned_offsets = pinned_offsets)
    if fresh
        repair_crossings!(r.offsets, anchors, sizes, r.dropped, p;
                          min_len = p.min_segment_length, pin_mask = pin_mask)
    end
    return (; offsets = r.offsets, dropped = r.dropped, iter = r.iter, residual = r.residual)
end
