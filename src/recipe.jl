# recipe.jl — the TextRepel Makie recipe.

@recipe TextRepel (positions,) begin
    "Labels to place: Vector of String / LaTeXString / rich text."
    text = nothing

    # ── Physics (pixel space, anisotropic) ──
    "Label↔label repulsion strength (x, y)."
    force = (1.0, 1.0)
    "Label↔data-point repulsion strength (x, y)."
    force_point = (1.0, 1.0)
    "Spring pull back to the anchor (x, y)."
    force_pull = (0.01, 0.01)
    "Maximum solver iterations."
    max_iter = 2000
    "Constrain movement: :both, :x, or :y."
    only_move = :both

    # ── Spacing / dropping ──
    "Pixels of padding around each label box for the solver."
    box_padding = 4.0
    "Pixel halo around each data point."
    point_padding = 0.0
    "Drop labels overlapping more than this many others (Inf = keep all)."
    max_overlaps = Inf

    # ── Connector segments ──
    "Draw connector lines from points to displaced labels."
    segments = true
    "Suppress a connector if the label moved fewer than this many pixels."
    min_segment_length = 5.0
    "Connector color."
    segmentcolor = :gray60
    "Connector width."
    linewidth = 1.0

    # ── Background box (geom_label_repel style) ──
    "Draw a filled background box behind each label."
    background = false
    "Background fill color."
    backgroundcolor = (:white, 0.8)
    "Background stroke color."
    strokecolor = :gray70
    "Background stroke width."
    strokewidth = 0.0
    "Background corner radius (px)."
    cornerradius = 4.0

    # ── Text passthrough ──
    fontsize = @inherit fontsize
    font = @inherit font
    color = @inherit textcolor
    align = (:center, :center)
end

# (xs, ys) → Vector{Point2f}
Makie.convert_arguments(::Type{<:TextRepel}, x::AbstractVector{<:Real}, y::AbstractVector{<:Real}) =
    (Point2f.(x, y),)

function Makie.plot!(p::TextRepel)
    # 1. Project anchors data → scene-local pixels (compute-graph node).
    register_projected_positions!(p, Point3f;
        input_space = :data, output_space = :pixel,
        input_name = :positions, output_name = :px_anchors)

    # 2. Measure + solve. Recomputes when anchors/text/font/size or params change.
    solved = lift(p.px_anchors, p.text, p.fontsize, p.font,
                  p.force, p.force_point, p.force_pull, p.max_iter, p.only_move,
                  p.box_padding, p.point_padding, p.max_overlaps) do px, labels, fs, font,
                                                                     fr, frp, fpl, mi, om,
                                                                     bp, pp, mo
        anchors = [Point2f(q[1], q[2]) for q in px]
        sizes = measure_labels(labels, font, fs, 1.0)
        params = RepelParams(; force = Tuple(Float64.(fr)),
                               force_point = Tuple(Float64.(frp)),
                               force_pull = Tuple(Float64.(fpl)),
                               max_iter = Int(mi), only_move = Symbol(om),
                               box_padding = Float64(bp), point_padding = Float64(pp),
                               max_overlaps = Float64(mo))
        offsets, dropped = solve_repel(anchors, sizes, params)
        (; anchors, sizes, offsets, dropped)
    end

    # Expose offsets for testing / downstream use. NOTE: in Makie 0.24 `p.attributes`
    # is a ComputeGraph, not a dict — use `add_input!`, not `setindex!`.
    Makie.add_input!(p.attributes, :computed_offsets, lift(s -> s.offsets, solved))

    # 3. Render text at original DATA positions with per-label pixel offsets,
    #    filtering out dropped labels.
    keep_positions = lift(p.positions, solved) do pos, s
        Point2f[pos[i] for i in eachindex(pos) if !s.dropped[i]]
    end
    keep_text = lift(p.text, solved) do labels, s
        [labels[i] for i in eachindex(labels) if !s.dropped[i]]
    end
    keep_offsets = @lift Vec2f[$solved.offsets[i] for i in eachindex($solved.offsets) if !$solved.dropped[i]]

    text!(p, keep_positions;
        text = keep_text, offset = keep_offsets, markerspace = :pixel,
        fontsize = p.fontsize, font = p.font, color = p.color, align = p.align)

    return p
end
