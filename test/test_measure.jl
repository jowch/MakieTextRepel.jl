# test_measure.jl
using CairoMakie   # provides a Makie backend so rendering paths work
using MakieTextRepel: measure_labels
using GeometryBasics

@testset "measure_labels" begin
    font = "TeX Gyre Heros Makie"

    # plain strings: matches Makie.text_bb to floating point
    sizes = measure_labels(["Hi", "Mauna Kea"], font, 24.0, 1.0)
    @test length(sizes) == 2
    for (s, str) in zip(sizes, ["Hi", "Mauna Kea"])
        bb = Makie.text_bb(str, Makie.to_font(font), 24f0)
        @test s[1] ≈ Makie.widths(bb)[1] atol = 1e-3
        @test s[2] ≈ Makie.widths(bb)[2] atol = 1e-3
    end

    # rich text: returns a finite, positive box
    rsize = only(measure_labels([rich("H", subscript("2"), "O")], font, 24.0, 1.0))
    @test all(isfinite, rsize)
    @test rsize[1] > 0 && rsize[2] > 0
end
