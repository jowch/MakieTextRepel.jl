using MakieTextRepel
using Test

@testset "MakieTextRepel.jl" begin
    include("test_geometry.jl")
    include("test_solver.jl")
    include("test_connectors.jl")
    include("test_measure.jl")
    include("test_integration.jl")
    include("test_annotation_algorithm.jl")
end
