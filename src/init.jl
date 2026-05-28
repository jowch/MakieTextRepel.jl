# init.jl — Imhof-preferred slot selection for label initialization.

"""Imhof preference order (TR most preferred; TL least). See Imhof 1962, Liao et al. 2024."""
const IMHOF_ORDER = (:TR, :R, :T, :BR, :L, :BL, :B, :TL)

"""
Offset (label center − anchor) for one of the 8 Imhof slots, positioned so the
anchor lies just outside the axis-aligned padded box of the label. `size` is
the unpadded label size (w, h); `p` is `point_padding`.
"""
function slot_offset(slot::Symbol, size::Vec2f, p::Real)
    p32 = Float32(p)
    w = size[1]
    h = size[2]
    hw = w / 2
    hh = h / 2
    slot === :TR && return Vec2f( p32 + hw,  p32 + hh)
    slot === :R  && return Vec2f( p32 + hw,  0f0)
    slot === :T  && return Vec2f( 0f0,       p32 + hh)
    slot === :BR && return Vec2f( p32 + hw, -p32 - hh)
    slot === :L  && return Vec2f(-p32 - hw,  0f0)
    slot === :BL && return Vec2f(-p32 - hw, -p32 - hh)
    slot === :B  && return Vec2f( 0f0,      -p32 - hh)
    slot === :TL && return Vec2f(-p32 - hw,  p32 + hh)
    error("unknown slot: $slot")
end
