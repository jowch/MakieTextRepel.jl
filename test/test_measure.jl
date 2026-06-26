# test_measure.jl
using CairoMakie   # provides a Makie backend so rendering paths work
using MakieTextRepel: measure_labels, measure_one
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

@testset "rich-text robustness (new render-free path)" begin
    font = "TeX Gyre Heros Makie"

    # rich("") crashes the old Scene render on this Makie version; the new
    # measure_bounds path returns a degenerate-but-finite box.
    esize = only(measure_labels([rich("")], font, 24.0, 1.0))
    @test all(isfinite, esize)
    @test esize[1] >= 0 && esize[2] >= 0

    # Unsupported label types raise a clear ArgumentError (not an opaque
    # Scene/MethodError). This also proves the Scene catch-all is gone.
    @test_throws ArgumentError measure_one(:Hello, font, 24.0, 1.0)
end

@testset "rich-text golden vs old Scene render" begin
    font = "TeX Gyre Heros Makie"

    # Replicates the pre-#6 Scene algorithm exactly; returns (w, h) in LOGICAL
    # pixels. Valid only for inputs the old path could render (NOT rich("")).
    function old_measure(label, fnt, fontsize)
        sc = Scene(size = (10, 10))
        t  = text!(sc, Point2f(0, 0); text = label, font = Makie.to_font(fnt),
                   fontsize = Float32(fontsize))
        Makie.update_state_before_display!(sc)
        w = Makie.widths(Makie.full_boundingbox(t, :pixel))
        return Vec2f(w[1], w[2])
    end

    # Representative rich-text constructs, measured at ppu = 1.
    cases = [
        rich("Hello, world"),                                    # simple
        rich("H", subscript("2"), "O"),                          # subscript
        rich("x", superscript("2")),                             # superscript
        rich("big ", rich("small"; fontsize = 12.0)),            # mixed size
        rich("plain ", rich("other"; font = "TeX Gyre Heros Makie Bold")),  # mixed font
        rich("line one\nline two"),                              # multi-line (ppu=1 only)
        rich(" "),                                               # whitespace-only
    ]
    for lbl in cases
        got  = only(measure_labels([lbl], font, 24.0, 1.0))
        want = old_measure(lbl, font, 24.0)
        @test got[1] ≈ want[1] atol = 0.5 rtol = 2e-3
        @test got[2] ≈ want[2] atol = 0.5 rtol = 2e-3
    end

    # Heterogeneous vector: each element dispatches independently.
    hetero = ["Mauna Kea", rich("H", subscript("2"), "O")]
    sizes  = measure_labels(hetero, font, 24.0, 1.0)
    for (got, lbl) in zip(sizes, hetero)
        want = old_measure(lbl, font, 24.0)
        @test got[1] ≈ want[1] atol = 0.5 rtol = 2e-3
        @test got[2] ≈ want[2] atol = 0.5 rtol = 2e-3
    end

    # ppu != 1: single-line rich post-scales exactly. The old :pixel bbox is
    # ppu-independent, so old_measure(...) .* 2 reproduces the old * ppu, and a
    # single line has no line-drop term, so the post-scale is exact.
    # NOTE: multi-line at ppu!=1 is intentionally NOT golden-tested here — the
    # 20px line-drop stub does not scale with ppu (a known limitation).
    sl    = rich("x", superscript("2"))
    got2  = only(measure_labels([sl], font, 24.0, 2.0))
    want2 = old_measure(sl, font, 24.0) .* 2.0
    @test got2[1] ≈ want2[1] atol = 0.5 rtol = 2e-3
    @test got2[2] ≈ want2[2] atol = 0.5 rtol = 2e-3

    # LaTeXString is an AbstractString → routes to the string path (method 1),
    # not the throwing catch-all. (Behavior is unchanged from before #6.)
    lstr  = Makie.LaTeXStrings.LaTeXString("x^2")
    lsize = only(measure_labels([lstr], font, 24.0, 1.0))
    @test all(isfinite, lsize)
    @test lsize[1] > 0 && lsize[2] > 0
end

@testset "font symbol resolves without warning (regression)" begin
    # The default `@inherit font` is the symbol :regular. It must resolve through
    # the theme `fonts` collection (:regular => "TeX Gyre Heros Makie"), NOT stringify
    # to "regular" — the literal-name lookup fails and warns ("Could not find font
    # regular, using TeX Gyre Heros Makie"). It still falls back to the same font, so
    # equal sizes alone don't catch it; assert the *absence* of the warning too.
    # The `==` is deliberate (not `≈`): :regular must resolve to the *same* font as
    # the literal name, so the boxes are byte-identical — that exactness is the
    # assertion. Don't relax to `≈`; the warning check below is the real guard.
    @test measure_labels(["Hi", "Mauna Kea"], :regular, 24.0, 1.0) ==
          measure_labels(["Hi", "Mauna Kea"], "TeX Gyre Heros Makie", 24.0, 1.0)
    @test_logs min_level = Base.CoreLogging.Warn measure_labels(["Hi"], :regular, 24.0, 1.0)
    @test_logs min_level = Base.CoreLogging.Warn measure_labels(["Hi"], :bold, 24.0, 1.0)
end

@testset "measure_labels public surface (#26)" begin
    font = "TeX Gyre Heros Makie"
    # Exported so warm_solve consumers have a fully public labels -> sizes path.
    @test :measure_labels in names(MakieTextRepel)
    # 3-arg convenience form measures at px_per_unit = 1.0 (identical to the 4-arg form).
    @test measure_labels(["Hi", "Mauna Kea"], font, 24.0) ==
          measure_labels(["Hi", "Mauna Kea"], font, 24.0, 1.0)
end
