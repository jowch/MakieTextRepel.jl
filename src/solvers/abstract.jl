# solvers/abstract.jl — Internal cluster-solver interface (v0.3 candidate for export).

"""
Marker type for cluster fallback solvers. Concrete subtypes implement
`solve_cluster(s, anchors, sizes, initial_offsets, bounds)` returning
`(offsets::Vector{Vec2f}, dropped::BitVector)`. Internal in v0.2; will be
exposed publicly when a second implementation lands (see GitHub issue #8).
"""
abstract type AbstractClusterSolver end

function solve_cluster end
