# recipe.jl — TextRepel Makie recipe.

@recipe TextRepel (positions,) begin
    "Labels to place: Vector of String / LaTeXString / rich text."
    text = nothing

    # ── Physics (pixel space, anisotropic) ──
    "Label↔label repulsion strength (x, y). Inert under the default ProjectionSolver (no force loop / zero-overlap guarantee); affects only the in-tree ForceSolver."
    force = (1.0, 1.0)
    "Label↔data-point repulsion strength (x, y). Inert under the default ProjectionSolver (no force loop / zero-overlap guarantee); affects only the in-tree ForceSolver."
    force_point = (1.0, 1.0)
    "Spring pull back to the anchor (x, y). Inert under the default ProjectionSolver (no force loop / zero-overlap guarantee); affects only the in-tree ForceSolver."
    force_pull = (0.01, 0.01)
    "Maximum solver iterations."
    max_iter = 2000
    "Constrain movement: :both, :x, or :y."
    only_move = :both

    # ── Spacing / dropping ──
    "Pixels of padding around each label box for the solver."
    box_padding = 4.0
    "Pixel marker-clearance: minimum gap from every scatter marker to the nearest label text edge, enforced after legalize (own and foreign markers). Also the connector anchor-end trim. Set to `marker_radius + small_gap`, or use `markersize`. (Under the in-tree ForceSolver it is the point-repulsion halo radius.)"
    point_padding = 5.0
    "Size of the SIBLING scatter marker, if any. `textrepel!` draws NO markers itself; this only tells the solver how much to clear. When set (scalar), overrides `point_padding` with `markersize/2 + 0.5`. Assumes a disc marker in `markerspace = :pixel` (the scatter default); for other markers set `point_padding` directly. `nothing` = use `point_padding`."
    markersize = nothing
    "Warm-start offsets (`Vector{Vec2f}`, pixel space), one per label, to relax from instead of seeding a fresh placement; `nothing` = fresh placement each solve. For animated plots that re-solve per frame, feed the previous frame's `computed_offsets` here. Length must equal the number of labels."
    init_state = nothing
    "Extra keep-out boxes (`Vector{Rect2f}`, pixel space) the solver must place labels clear of, in addition to the data markers."
    obstacles = Rect2f[]
    "Drop labels overlapping more than this many others (Inf = keep all). Inert under the default ProjectionSolver (no force loop / zero-overlap guarantee); affects only the in-tree ForceSolver."
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
    # 1. Project anchors to pixel space.
    register_projected_positions!(p, Point3f;
        input_space = :data, output_space = :pixel,
        input_name = :positions, output_name = :px_anchors)

    # Clamp region: viewport size only. Viewport Rect2i origin is figure-relative but
    # :pixel anchors are scene-local (axis lower-left = 0,0), so discard the origin.
    bounds_obs = lift(Makie.viewport(Makie.parent_scene(p))) do vp
        Rect2f(0, 0, Float32.(widths(vp))...)
    end

    # 2a. Measure label boxes. Keyed on text/fontsize/font only (#25): not re-run on
    #     position or solve-param changes (animation reuse).
    measured_sizes = lift(p.text, p.fontsize, p.font) do labels, fs, font
        measure_labels(labels, font, fs, 1.0)
    end

    # 2b. Solve. Reuses cached `measured_sizes`; re-runs on anchor/param changes.
    #     `sizes` threaded into result tuple for downstream nodes.
    solved = lift(p.px_anchors, measured_sizes,
                  p.force, p.force_point, p.force_pull, p.max_iter, p.only_move,
                  p.box_padding, p.point_padding, p.max_overlaps, bounds_obs,
                  p.min_segment_length, p.markersize, p.init_state, p.obstacles) do px, sizes,
                                                          fr, frp, fpl, mi, om,
                                                          bp, pp, mo, bnds, ml, ms, is, obs
        anchors = [Point2f(q[1], q[2]) for q in px]
        # markersize overrides point_padding when set.
        eff_pp = if ms === nothing
            Float64(pp)
        elseif ms isa Real
            Float64(ms) / 2 + 0.5
        else
            throw(ArgumentError("textrepel!: `markersize` must be a scalar Real or nothing; got $(typeof(ms)). Per-point marker sizes are not supported — set `point_padding` directly."))
        end
        params = RepelParams(; force = Tuple(Float64.(fr)),
                               force_point = Tuple(Float64.(frp)),
                               force_pull = Tuple(Float64.(fpl)),
                               max_iter = Int(mi), only_move = Symbol(om),
                               box_padding = Float64(bp), point_padding = eff_pp,
                               max_overlaps = Float64(mo), bounds = bnds,
                               min_segment_length = Float64(ml))
        # `bounds_obs` always yields a Rect2f; `bnds` is never nothing here.
        # `obstacles` defaults to Rect2f[], never nothing — no nothing-guard needed.
        is_v  = is === nothing ? nothing : Vector{Vec2f}(is)
        obs_v = Vector{Rect2f}(obs)
        sol = solve_cluster(ProjectionSolver(params), anchors, sizes, bnds;
                            init_state = is_v, obstacles = obs_v)
        (; anchors, sizes, offsets = sol.offsets, dropped = sol.dropped, params)
    end

    # Expose offsets for testing/downstream use.
    # Gotcha: p.attributes is a ComputeGraph (Makie 0.24), not a dict — use add_input!, not setindex!.
    Makie.add_input!(p.attributes, :computed_offsets, lift(s -> s.offsets, solved))
    Makie.add_input!(p.attributes, :computed_anchors, lift(s -> s.anchors, solved))
    Makie.add_input!(p.attributes, :computed_sizes,   lift(s -> s.sizes,   solved))
    Makie.add_input!(p.attributes, :computed_dropped, lift(s -> s.dropped, solved))
    Makie.add_input!(p.attributes, :computed_params,  lift(s -> s.params,  solved))

    # 3. Render text at data positions with pixel offsets; skip dropped labels.
    keep_positions = lift(p.positions, solved) do pos, s
        Point2f[pos[i] for i in eachindex(pos) if !s.dropped[i]]
    end
    keep_text = lift(p.text, solved) do labels, s
        [labels[i] for i in eachindex(labels) if !s.dropped[i]]
    end
    keep_offsets = @lift Vec2f[$solved.offsets[i] for i in eachindex($solved.offsets) if !$solved.dropped[i]]

    # 3a. Background boxes (pixel space). cornerradius is wired but unused; draws plain Rect2f.
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

    # 4. Connector segments (pixel space).
    seg_points = lift(solved, p.min_segment_length, p.box_padding, p.point_padding, p.segments) do s, ml, bp, pp, on
        on || return Point2f[]
        build_connectors(s.anchors, s.offsets, s.sizes, s.dropped,
                         Float64(ml), Float64(bp); point_padding = Float64(pp))
    end
    linesegments!(p, seg_points; space = :pixel,
        color = p.segmentcolor, linewidth = p.linewidth, visible = p.segments)

    return p
end

# Gotcha: axis autolimits must see only the data anchors. The linear-scale path
# uses boundingbox, not data_limits — override BOTH or pixel glyph extents leak.
Makie.data_limits(p::TextRepel) = Makie.data_limits(p.plots[1])
Makie.data_limits(p::TextRepel{<:Tuple{<:AbstractVector{<:Point}}}) = Rect3d(p[1][])
Makie.boundingbox(p::TextRepel, space::Symbol) =
    Makie.apply_transform_and_model(p, Makie.data_limits(p))
