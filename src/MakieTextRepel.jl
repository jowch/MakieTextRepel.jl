module MakieTextRepel

using Makie
using GeometryBasics
using LinearAlgebra
import TextMeasure

export textrepel, textrepel!

include("geometry.jl")
include("solver.jl")
include("solvers/abstract.jl")
include("solvers/force.jl")
include("voronoi.jl")
include("init.jl")
include("connectors.jl")
include("crossings.jl")
include("measure.jl")
include("recipe.jl")
include("annotation_algorithm.jl")

end # module MakieTextRepel
