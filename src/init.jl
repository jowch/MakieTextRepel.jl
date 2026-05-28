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

"""
Initial offsets for each anchor: pick the highest-preference Imhof slot whose
padded box fits inside the anchor's Voronoi cell; fall back to TR if none fit
or the cell is `nothing`. Apply `_constrain(offset, params.only_move)` to
respect axis-lock semantics from the first iteration.

Pure function of (anchors, sizes, cells, params). Same inputs → same outputs.
"""
function initial_offsets(anchors::Vector{Point2f}, sizes::Vector{Vec2f},
                         cells::Vector{<:Union{GeometryBasics.Polygon, Nothing}},
                         params)
    n = length(anchors)
    offsets = Vector{Vec2f}(undef, n)
    pad = Float32(params.box_padding)
    p = params.point_padding
    for i in 1:n
        cell = cells[i]
        chosen = :TR
        if cell !== nothing
            for slot in IMHOF_ORDER
                candidate = slot_offset(slot, sizes[i], p)
                padded_size = sizes[i] .+ 2pad
                box = box_at(anchors[i], candidate, padded_size)
                if box_inside_polygon(box, cell)
                    chosen = slot
                    break
                end
            end
        end
        raw_off = slot_offset(chosen, sizes[i], p)
        offsets[i] = _constrain(raw_off, params.only_move)
    end
    return offsets
end
