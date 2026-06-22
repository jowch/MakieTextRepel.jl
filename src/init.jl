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

When `pin_mask === nothing` (the recipe path) the result is unchanged. When
`pin_mask`/`pinned_offsets` are provided, indices where `pin_mask[i]` is true
are seeded directly at `pinned_offsets[i]`, skipping both the slot search and
`_constrain` (pinned offsets are taken verbatim).

Pure function of (anchors, sizes, cells, params, pin_mask, pinned_offsets).
Same inputs → same outputs.
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
    # Coincidence: a label sharing its (x,y) with ≥1 other has no Voronoi cell and
    # would otherwise get the same TR fallback slot as its twin → collapse. Tag those
    # so we can fan them out by index. Distinct-but-degenerate (e.g. collinear) labels
    # are NOT tagged — they keep the plain TR fallback (preserves recipe byte-identity
    # and the distinct-collinear crossing layout).
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
        # Fan out genuinely coincident anchors (cell === nothing AND shares coords):
        # deterministic index-keyed golden-angle direction at the slot's radius. Pure
        # function of index — preserves determinism. Single/distinct labels untouched.
        if cell === nothing && coord_counts[(anchors[i][1], anchors[i][2])] > 1
            rad = sqrt(raw_off[1]^2 + raw_off[2]^2)
            θ   = _GOLDEN_ANGLE * Float32(i)
            raw_off = Vec2f(rad * cos(θ), rad * sin(θ))
        end
        offsets[i] = _constrain(raw_off, params.only_move)
    end
    return offsets
end
