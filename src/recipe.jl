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
    "Pixel halo around each data point — used both by the solver (repulsion halo) and by the connector layer (gap between the marker and the start of the leader line)."
    point_padding = 2.0
    "Drop labels overlapping more than this many others (Inf = keep all)."
    max_overlaps = Inf

    # ── Connector segments ──
    "Draw connector lines from points to displaced labels."
    segments = true
    "Suppress a connector if its *visible* length (anchor end trimmed by `point_padding`, label end clipped to box face) is fewer than this many pixels."
    min_segment_length = 2.0
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

    # Clamp region: the axis data-area viewport, SIZE ONLY. The viewport Rect2i carries
    # a figure-relative origin, but :pixel anchors are scene-local (origin at the axis
    # lower-left), so we use (0,0)–widths and discard the origin.
    bounds_obs = lift(Makie.viewport(Makie.parent_scene(p))) do vp
        Rect2f(0, 0, Float32.(widths(vp))...)
    end

    # 2. Measure + solve. Recomputes when anchors/text/font/size or params change.
    solved = lift(p.px_anchors, p.text, p.fontsize, p.font,
                  p.force, p.force_point, p.force_pull, p.max_iter, p.only_move,
                  p.box_padding, p.point_padding, p.max_overlaps, bounds_obs) do px, labels, fs, font,
                                                                                 fr, frp, fpl, mi, om,
                                                                                 bp, pp, mo, bnds
        anchors = [Point2f(q[1], q[2]) for q in px]
        sizes = measure_labels(labels, font, fs, 1.0)
        params = RepelParams(; force = Tuple(Float64.(fr)),
                               force_point = Tuple(Float64.(frp)),
                               force_pull = Tuple(Float64.(fpl)),
                               max_iter = Int(mi), only_move = Symbol(om),
                               box_padding = Float64(bp), point_padding = Float64(pp),
                               max_overlaps = Float64(mo), bounds = bnds)
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

    # 3a. Optional background boxes (drawn beneath text), pixel space.
    # TODO(rounded-corners): cornerradius attribute is wired but unused here; v1 draws plain Rect2f.
    box_rects = lift(solved, p.background, p.box_padding) do s, bg, bp
        bg || return Rect2f[]
        pad = Float32(bp)
        Rect2f[box_at(s.anchors[i], s.offsets[i], s.sizes[i] .+ 2pad)
               for i in eachindex(s.anchors) if !s.dropped[i]]
    end
    poly!(p, box_rects; space = :pixel,
        color = p.backgroundcolor, strokecolor = p.strokecolor,
        strokewidth = p.strokewidth, visible = p.background)

    text!(p, keep_positions;
        text = keep_text, offset = keep_offsets, markerspace = :pixel,
        fontsize = p.fontsize, font = p.font, color = p.color, align = p.align)

    # 4. Connector segments (pixel space; coexists with data-space text anchors).
    seg_points = lift(solved, p.min_segment_length, p.box_padding, p.point_padding, p.segments) do s, ml, bp, pp, on
        on || return Point2f[]
        build_connectors(s.anchors, s.offsets, s.sizes, s.dropped,
                         Float64(ml), Float64(bp); point_padding = Float64(pp))
    end
    linesegments!(p, seg_points; space = :pixel,
        color = p.segmentcolor, linewidth = p.linewidth, visible = p.segments)

    return p
end

# Axis autolimits must track the data anchors only — the pixel-space offset
# children (text, boxes, connectors) must not inflate the limits. The axis's
# linear-scale path uses `boundingbox(scene, exclude)`, NOT `data_limits`, so we
# must override BOTH (mirroring Makie's own `textlabel` recipe), otherwise the
# `text!` child's pixel glyph extents leak into the data limits.
Makie.data_limits(p::TextRepel) = Makie.data_limits(p.plots[1])
Makie.data_limits(p::TextRepel{<:Tuple{<:AbstractVector{<:Point}}}) = Rect3d(p[1][])
Makie.boundingbox(p::TextRepel, space::Symbol) =
    Makie.apply_transform_and_model(p, Makie.data_limits(p))
