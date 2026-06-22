# init.jl — Imhof-preferred slot selection for label initialization.

"""Imhof preference order (TR most preferred; TL least). Imhof 1962, Liao et al. 2024."""
const IMHOF_ORDER = (:TR, :R, :T, :BR, :L, :BL, :B, :TL)

"""
Offset (label center − anchor) for one of the 8 Imhof slots, placing the anchor
just outside the padded label box. `size` is unpadded (w, h); `p` is `point_padding`.
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
    initial_offsets(anchors, sizes, cells, params; pin_mask, pinned_offsets)
        -> Vector{Vec2f}

Seed each label at the highest-preference Imhof slot whose padded box fits inside
the anchor's Voronoi cell; fall back to TR when none fit or `cell === nothing`.
`_constrain(offset, params.only_move)` applied to enforce axis-lock from the first
iteration.

`pin_mask[i] == true`: seed at `pinned_offsets[i]`, skip slot search and `_constrain`.

Pure: same inputs → same outputs.
"""
function initial_offsets(anchors::Vector{Point2f}, sizes::Vector{Vec2f},
                         cells::Vector{<:Union{GeometryBasics.Polygon, Nothing}},
                         params;
                         pin_mask::Union{Nothing,BitVector} = nothing,
                         pinned_offsets::Vector{Vec2f}       = Vec2f[])
    n = length(anchors)
    if pin_mask !== nothing
        length(pin_mask) == n || throw(DimensionMismatch(
            "pin_mask length $(length(pin_mask)) does not match anchors length $n"))
        length(pinned_offsets) == n || throw(DimensionMismatch(
            "pinned_offsets length $(length(pinned_offsets)) does not match anchors length $n"))
    end
    offsets = Vector{Vec2f}(undef, n)
    pad = Float32(params.box_padding)
    p = params.point_padding
    # Coincident anchors share (x,y), have no Voronoi cell, and would all get the
    # same TR fallback → collapse. Fan them out by index via golden-angle direction.
    # Distinct-but-degenerate (e.g. collinear) labels keep plain TR fallback.
    coord_counts = Dict{Tuple{Float32,Float32},Int}()
    for a in anchors
        k = (a[1], a[2]); coord_counts[k] = get(coord_counts, k, 0) + 1
    end
    for i in 1:n
        if pin_mask !== nothing && pin_mask[i]
            offsets[i] = pinned_offsets[i]      # pinned: seed at the fixed offset, skip slot search
            continue
        end
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
        # Fan-out for genuinely coincident anchors: deterministic golden-angle direction
        # at the slot's radius, keyed on index. Distinct labels untouched.
        if cell === nothing && coord_counts[(anchors[i][1], anchors[i][2])] > 1
            rad = sqrt(raw_off[1]^2 + raw_off[2]^2)
            θ   = _GOLDEN_ANGLE * Float32(i)
            raw_off = Vec2f(rad * cos(θ), rad * sin(θ))
        end
        offsets[i] = _constrain(raw_off, params.only_move)
    end
    return offsets
end
