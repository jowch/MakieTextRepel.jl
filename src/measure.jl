# measure.jl — pixel box sizes for labels, render-free via TextMeasure.
# Plain strings (and LaTeXString) go through the layout engine; rich text
# (Makie.RichText) through measure_bounds. No Scene/render is allocated.

"""Resolve a Makie font attribute to something TextMeasure/`text_bb` accept."""
_resolve_font(f) = Makie.to_font(f)
_resolve_font(f::Symbol) = Makie.to_font(String(f))

"""Measure every label, returning a `Vector{Vec2f}` of (width, height) in pixels."""
measure_labels(labels, font, fontsize::Real, px_per_unit::Real) =
    [measure_one(lbl, font, Float64(fontsize), Float64(px_per_unit)) for lbl in labels]

# Plain strings (and LaTeXString) → TextMeasure layout engine, render-free.
# The string path passes ppu straight into the backend (layout honors it).
# TODO(glyph-fallback): route through text_bb when the resolved font lacks the label's glyphs.
function measure_one(label::AbstractString, font, fontsize::Float64, ppu::Float64)
    f = _resolve_font(font)
    backend = TextMeasure.MakieBackend(; font = f, fontsize = fontsize, px_per_unit = ppu)
    lay = TextMeasure.layout(TextMeasure.prepare(backend, String(label)))
    return Vec2f(lay.size[1], lay.size[2])
end

# Rich text → TextMeasure measure_bounds, render-free. measure_bounds requires
# px_per_unit == 1 (its line-drop stub does not scale), so we measure at 1 and
# apply ppu as a post-scale — matching the old full_boundingbox(:pixel) * ppu.
function measure_one(label::Makie.RichText, font, fontsize::Float64, ppu::Float64)
    f = _resolve_font(font)
    backend = TextMeasure.MakieBackend(; font = f, fontsize = fontsize, px_per_unit = 1.0)
    tb = TextMeasure.measure_bounds(backend, label)
    return Vec2f(tb.size[1] * ppu, tb.size[2] * ppu)
end

# Any other label type → a clear error. The old Scene path also failed on these
# (Makie has no convert_text_string! for Symbol/Number); this just makes it legible.
measure_one(label, font, fontsize::Float64, ppu::Float64) =
    throw(ArgumentError("textrepel! labels must be String, LaTeXString, or " *
                        "Makie.RichText; got $(typeof(label))"))
