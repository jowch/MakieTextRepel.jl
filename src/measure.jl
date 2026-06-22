# measure.jl — pixel box sizes for labels, render-free via TextMeasure.
# Plain strings (and LaTeXString) go through the layout engine; rich text
# (Makie.RichText) through measure_bounds. No Scene/render is allocated.

"""Resolve a Makie font attribute to a font object TextMeasure accepts."""
_resolve_font(f) = Makie.to_font(f)
_resolve_font(f::Symbol) = Makie.to_font(String(f))

"""
    measure_labels(labels, font, fontsize) -> Vector{Vec2f}
    measure_labels(labels, font, fontsize, px_per_unit) -> Vector{Vec2f}

Render-free per-label text measurement: each label's unpadded `(width, height)` in
pixels, as a `Vector{Vec2f}` in input order. `labels` may be `String`/`LaTeXString` or
`Makie.RichText`; `font` is any Makie font attribute (a name `String`/`Symbol` or a font
object); `fontsize` is in pixels. No `Scene` is allocated — measurement goes through
TextMeasure.jl.

The 3-argument form measures at `px_per_unit = 1.0` (pixel space), which is what the
`textrepel!` recipe and pixel-space [`warm_solve`](@ref) consumers want; pass a 4th
positional `px_per_unit` to scale. This is the public path for producing the `sizes`
argument of `warm_solve`, e.g.
`warm_solve(anchors, measure_labels(labels, font, fontsize), bounds; …)`.
"""
measure_labels(labels, font, fontsize::Real, px_per_unit::Real) =
    [measure_one(lbl, font, Float64(fontsize), Float64(px_per_unit)) for lbl in labels]

# 3-arg default: px_per_unit = 1.0 (pixel space). Internal callers pass ppu positionally.
measure_labels(labels, font, fontsize::Real) = measure_labels(labels, font, fontsize, 1.0)

# Plain strings (and LaTeXString) → TextMeasure layout engine. ppu passed to backend.
# TODO(glyph-fallback): route through text_bb when the resolved font lacks the label's glyphs.
function measure_one(label::AbstractString, font, fontsize::Float64, ppu::Float64)
    f = _resolve_font(font)
    backend = TextMeasure.MakieBackend(; font = f, fontsize = fontsize, px_per_unit = ppu)
    lay = TextMeasure.layout(TextMeasure.prepare(backend, String(label)))
    return Vec2f(lay.size[1], lay.size[2])
end

# Rich text → TextMeasure measure_bounds. measure_bounds requires px_per_unit == 1
# (line-drop stub does not scale); measure at 1 and post-scale by ppu.
function measure_one(label::Makie.RichText, font, fontsize::Float64, ppu::Float64)
    f = _resolve_font(font)
    backend = TextMeasure.MakieBackend(; font = f, fontsize = fontsize, px_per_unit = 1.0)
    tb = TextMeasure.measure_bounds(backend, label)
    return Vec2f(tb.size[1] * ppu, tb.size[2] * ppu)
end

# Any other label type → clear error (String, LaTeXString, and RichText are the only supported types).
measure_one(label, font, fontsize::Float64, ppu::Float64) =
    throw(ArgumentError("textrepel! labels must be String, LaTeXString, or " *
                        "Makie.RichText; got $(typeof(label))"))
