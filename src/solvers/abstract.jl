# solvers/abstract.jl — cluster-solver interface (internal; export gated on issue #8).

"""
Abstract type for cluster placement solvers. Subtypes implement

    solve_cluster(s, anchors, sizes, bounds;
                  init_state = nothing, pin_mask = nothing,
                  pinned_offsets = Vec2f[], obstacles = Rect2f[])
        -> (; offsets::Vector{Vec2f}, dropped::BitVector, iter::Int, residual::Float32)

`init_state === nothing` ⇒ fresh placement (init + crossing repair inside
`solve_cluster`); given `init_state` ⇒ warm-start (legalize only). Callers must
not run init/repair outside `solve_cluster`. `ProjectionSolver` output is
**deterministic**: identical inputs → byte-identical `offsets`/`dropped` (no RNG;
DelaunayTriangulation seeded `MersenneTwister(0)` over lex-sorted points;
coincident anchors spiral along a fixed golden-angle fan). Public stateless face:
`warm_solve` (issue #8 for seam export).
"""
abstract type AbstractClusterSolver end

function solve_cluster end
