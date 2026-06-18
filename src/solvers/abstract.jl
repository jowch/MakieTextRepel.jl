# solvers/abstract.jl — Internal cluster-solver interface (candidate for export, issue #8).

"""
Marker type for cluster placement solvers. A concrete subtype owns the **entire**
placement strategy and implements

    solve_cluster(s, anchors, sizes, bounds;
                  init_state = nothing, pin_mask = nothing,
                  pinned_offsets = Vec2f[], obstacles = Rect2f[])
        -> (; offsets::Vector{Vec2f}, dropped::BitVector, iter::Int, residual::Float32)

`init_state === nothing` ⇒ fresh placement (the solver does its own init + crossing
repair); a given `init_state` ⇒ relax (warm-start, solve only). Callers must NOT
perform init/placement/repair outside `solve_cluster`. Under the default
`ProjectionSolver` the result is **deterministic** — identical inputs yield
byte-identical `offsets`/`dropped` (no RNG; DelaunayTriangulation is seeded with
`MersenneTwister(0)` over lexicographically sorted points). Internal for now; the
public stateless face is `warm_solve` (the seam itself is exposed when a second
strategy lands — see GitHub issue #8).
"""
abstract type AbstractClusterSolver end

function solve_cluster end
