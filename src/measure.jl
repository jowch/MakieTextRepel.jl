# measure.jl — pixel box sizes for labels. Plain text via TextMeasure (render-free);
# rich text via Makie's full_boundingbox after a cheap render.

"""Resolve a Makie font attribute to something TextMeasure/`text_bb` accept."""
_resolve_font(f) = Makie.to_font(f)
_resolve_font(f::Symbol) = Makie.to_font(String(f))

"""Measure every label, returning a `Vector{Vec2f}` of (width, height) in pixels."""
measure_labels(labels, font, fontsize::Real, px_per_unit::Real) =
    [measure_one(lbl, font, Float64(fontsize), Float64(px_per_unit)) for lbl in labels]

# Plain strings (and LaTeXString) → TextMeasure, render-free.
# TODO(glyph-fallback): route through text_bb when the resolved font lacks the label's glyphs.
function measure_one(label::AbstractString, font, fontsize::Float64, ppu::Float64)
    f = _resolve_font(font)
    backend = TextMeasure.MakieBackend(; font = f, fontsize = fontsize, px_per_unit = ppu)
    lay = TextMeasure.layout(TextMeasure.prepare(backend, String(label)))
    return Vec2f(lay.size[1], lay.size[2])
end

# Rich text (and any non-string) → cheap render + full_boundingbox(:pixel).
function measure_one(label, font, fontsize::Float64, ppu::Float64)
    f = _resolve_font(font)
    scene = Scene(size = (10, 10))
    t = text!(scene, Point2f(0, 0); text = label, font = f, fontsize = Float32(fontsize))
    Makie.update_state_before_display!(scene)
    bb = Makie.full_boundingbox(t, :pixel)
    w = Makie.widths(bb)
    return Vec2f(w[1] * ppu, w[2] * ppu)
end
