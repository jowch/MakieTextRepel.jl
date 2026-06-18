module MakieTextRepel

using Makie
using GeometryBasics
using LinearAlgebra
import TextMeasure

export textrepel, textrepel!, warm_solve, measure_labels

# Shared config (consumed by both solvers)
include("params.jl")

# Pure geometric layers (Makie-free, GeometryBasics only)
include("geometry.jl")        # AABB primitives: box_at, overlap/point push, clipping, clamp
include("voronoi.jl")         # per-anchor Voronoi cells + cell-fit predicate
include("init.jl")            # Imhof-slot initial placement (seeds both solvers)
include("connectors.jl")      # leader-line construction → linesegments! pairs
include("crossings.jl")       # leader-line crossing detection + 2-opt repair

# ProjectionSolver pure pipeline stages (the default solver composes these)
include("side_select.jl")     # greedy discrete Imhof-slot refinement
include("legalize.jl")        # Dykstra constraint-projection overlap removal
include("cost.jl")            # read-only placement-quality functional (label_cost)

# Force-directed model — the non-default fallback path
include("force_model.jl")     # solve_repel + helpers (formerly solver.jl)

# Cluster-solver seam + implementations
include("solvers/abstract.jl")    # AbstractClusterSolver interface
include("solvers/force.jl")       # ForceSolver — wraps force_model.jl
include("solvers/projection.jl")  # ProjectionSolver — the DEFAULT, composes the stages above

# Public stateless warm-start primitive (face over the internal seam)
include("warm_solve.jl")

# Measurement + Makie surfaces
include("measure.jl")
include("recipe.jl")
include("annotation_algorithm.jl")

end # module MakieTextRepel
