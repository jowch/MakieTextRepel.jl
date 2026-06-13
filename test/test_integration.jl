using CairoMakie
using MakieTextRepel

@testset "recipe declaration" begin
    # the recipe type and functions exist
    @test isdefined(MakieTextRepel, :TextRepel)
    @test isa(textrepel!, Function)

    # (xs, ys) convenience form converts to a positions vector without error
    fig = Figure()
    ax = Axis(fig[1, 1])
    pl = textrepel!(ax, [1.0, 2.0, 3.0], [1.0, 2.0, 3.0];
                    text = ["a", "b", "c"])
    @test pl isa MakieTextRepel.TextRepel
end

@testset "textrepel end-to-end (plain text)" begin
    fig = Figure(size = (400, 400))
    ax = Axis(fig[1, 1])
    pts = Point2f[(1, 1), (1.01, 1.01), (1.02, 0.99)]   # nearly coincident
    pl = textrepel!(ax, pts; text = ["alpha", "beta", "gamma"])

    # force the scene to lay out so the recipe's compute graph runs
    Makie.update_state_before_display!(fig.scene)

    offs = pl.computed_offsets[]
    @test length(offs) == 3
    @test all(o -> all(isfinite, o), offs)
    @test maximum(norm, offs) > 0          # crowded labels actually moved
end

@testset "connectors render" begin
    fig = Figure(size = (400, 400))
    ax = Axis(fig[1, 1])
    pts = Point2f[(1, 1), (1.005, 1.005)]   # very close → labels must move far
    pl = textrepel!(ax, pts; text = ["overlapping one", "overlapping two"],
                    segments = true, min_segment_length = 1.0)
    Makie.update_state_before_display!(fig.scene)

    # a LineSegments child plot exists and has an even number of endpoints
    seg_plots = filter(c -> c isa LineSegments, pl.plots)
    @test length(seg_plots) == 1
    @test iseven(length(seg_plots[1][1][]))
end

@testset "background boxes" begin
    fig = Figure(size = (400, 400))
    ax = Axis(fig[1, 1])
    pl = textrepel!(ax, Point2f[(1, 1), (2, 2)]; text = ["one", "two"],
                    background = true)
    Makie.update_state_before_display!(fig.scene)

    # a Poly child plot exists when background = true
    poly_plots = filter(c -> c isa Poly, pl.plots)
    @test length(poly_plots) == 1
end

@testset "visual artifact renders" begin
    fig = Figure(size = (600, 400))
    ax = Axis(fig[1, 1]; title = "textrepel demo")
    pts = Point2f[(1, 1), (1.1, 1.05), (1.05, 0.9), (2, 2), (2.05, 2.02)]
    scatter!(ax, pts; color = :tomato)
    textrepel!(ax, pts; text = ["alpha", "beta", "gamma", "delta", "epsilon"],
               segments = true)
    out = joinpath(@__DIR__, "output", "demo.png")
    mkpath(dirname(out))
    save(out, fig)
    @test isfile(out) && filesize(out) > 0
end

@testset "axis limits track data, not pixel offsets" begin
    fig = Figure(); ax = Axis(fig[1, 1])
    pts = Point2f[(1, 1), (1.1, 1.05), (1.05, 0.9), (2, 2), (2.05, 2.02)]
    pl = textrepel!(ax, pts; text = ["a", "b", "c", "d", "e"])

    # The recipe's own data_limits must report the anchor extent only.
    dl = Makie.data_limits(pl)
    @test dl.widths[1] < 5
    @test dl.widths[2] < 5

    # And — crucially — the ACTUAL axis autolimits must follow it. The axis uses
    # boundingbox(scene, exclude) on the linear path, so this catches a leak that
    # a data_limits-only check would miss. Data spans ~x∈[1,2.05], y∈[0.9,2.02];
    # limits must stay near the data, not blow up to the pixel scale (~100+).
    reset_limits!(ax)
    lims = ax.finallimits[]
    @test lims.widths[1] < 5
    @test lims.widths[2] < 5
end

@testset "connectors suppressed when clamp pins anchor inside its own label" begin
    # Tiny viewport (100×80) + wide label anchored at the data midpoint: the
    # label box (≈154 px wide) is over 3× wider than the axis viewport
    # (≈43 px). This is an OVER-CAPACITY scene for the 5px marker-clearance
    # floor (#21): there is no in-bounds placement that clears the anchor, so
    # the floor is waived and the clamp simply confines the oversized box to
    # bounds. The anchor, projecting to pixel (≈2, ≈1), ends up strictly inside
    # the clamped box (verified: box origin ≈(-111, 0.7), widths ≈(154, 24)).
    # The connector layer's strict-inside check (clip_to_box_edge → nothing)
    # then suppresses the segment. Survives the point_padding default bump
    # 2.0→5.0 precisely because the scene is too small for the new floor to
    # push the anchor clear — not because of any particular box size.
    fig = Figure(size = (100, 80)); ax = Axis(fig[1, 1])
    pts = Point2f[(0.5, 0.5)]
    pl = textrepel!(ax, pts; text = ["a-very-wide-label-name"],
                    segments = true, min_segment_length = 0.0)
    Makie.update_state_before_display!(fig.scene)

    seg_plots = filter(c -> c isa LineSegments, pl.plots)
    @test length(seg_plots) == 1
    @test isempty(seg_plots[1][1][])
end

@testset "clamping keeps labels inside the axis viewport" begin
    fig = Figure(size = (420, 360)); ax = Axis(fig[1, 1])
    pts = Point2f[(0, 0), (1, 0), (0, 1), (1, 1), (0.5, 0.5)]
    labels = ["alphalabel", "betalabel", "gammalabel", "deltalabel", "epsilonlabel"]
    pl = textrepel!(ax, pts; text = labels)
    Makie.update_state_before_display!(fig.scene)

    offs = pl.computed_offsets[]
    vp = Makie.widths(Makie.viewport(ax.scene)[])          # scene-local size
    sizes = MakieTextRepel.measure_labels(labels, pl.font[], pl.fontsize[], 1.0)
    pad = Float32(pl.box_padding[])
    for i in eachindex(pts)
        apx = Makie.project(ax.scene, :data, :pixel, pts[i])
        anchor = Point2f(apx[1], apx[2])
        box = MakieTextRepel.box_at(anchor, offs[i], sizes[i] .+ Vec2f(2pad))
        @test box.origin[1] >= -1.0
        @test box.origin[2] >= -1.0
        @test box.origin[1] + box.widths[1] <= vp[1] + 1.0
        @test box.origin[2] + box.widths[2] <= vp[2] + 1.0
    end
end

using MakieTextRepel: connector_for, find_crossings, label_cost, solve_cluster, ForceSolver

within_bounds(pos::Point2f, vp::Rect2f) =
    vp.origin[1] <= pos[1] <= vp.origin[1] + vp.widths[1] &&
    vp.origin[2] <= pos[2] <= vp.origin[2] + vp.widths[2]

@testset "pipeline invariants" begin
    # Three case fixtures spanning sparsity regimes.
    cases = [
        (name = "sparse",
         anchors = Point2f[(0.2, 0.2), (0.8, 0.2), (0.5, 0.8), (0.2, 0.8), (0.8, 0.8)],
         labels = ["a", "b", "c", "d", "e"],
         limits = (0, 1, 0, 1)),
        (name = "dense",
         anchors = Point2f[(0.5, 0.5), (0.51, 0.51), (0.49, 0.49), (0.5, 0.48)],
         labels = ["alpha", "beta", "gamma", "delta"],
         limits = (0.4, 0.6, 0.4, 0.6)),
        (name = "mixed",
         anchors = Point2f[(0.1, 0.1), (0.9, 0.9), (0.5, 0.51), (0.52, 0.49), (0.48, 0.5)],
         labels = ["isolated_a", "isolated_b", "clu1", "clu2", "clu3"],
         limits = (0, 1, 0, 1)),
    ]

    for case in cases
        @testset "$(case.name)" begin
            fig = Figure(size = (400, 400))
            ax = Axis(fig[1, 1], limits = case.limits)
            plt = textrepel!(ax, case.anchors; text = case.labels)
            Makie.update_state_before_display!(fig)

            offsets = plt.attributes[:computed_offsets][]
            anchors = plt.attributes[:computed_anchors][]
            sizes   = plt.attributes[:computed_sizes][]
            dropped = plt.attributes[:computed_dropped][]
            params  = plt.attributes[:computed_params][]
            min_len = plt.min_segment_length[]

            connectors = [connector_for(anchors[i], offsets[i], sizes[i], dropped[i], params, min_len)
                          for i in eachindex(offsets)]
            # Pixel-space viewport in axis-scene-local coordinates — same frame as `anchors`.
            vp = params.bounds
            @test all(isfinite, offsets)
            @test all(i -> dropped[i] || within_bounds(anchors[i] + offsets[i], vp), eachindex(offsets))
            # HARD guarantee: zero box overlap under ProjectionSolver.
            q = label_cost(anchors, sizes; offsets = offsets, bounds = vp, dropped = dropped,
                           box_padding = params.box_padding,
                           point_padding = params.point_padding,
                           min_segment_length = min_len)
            @test q.overlaps == 0
            # Crossings are best-effort under ProjectionSolver (repair precedes the final legalize):
            # assert no worse than the ForceSolver baseline on the same scene.
            rf = solve_cluster(ForceSolver(params), anchors, sizes, vp)
            force_conn = [connector_for(anchors[i], rf.offsets[i], sizes[i], rf.dropped[i], params, min_len)
                          for i in eachindex(rf.offsets)]
            @test length(find_crossings(connectors)) ≤ length(find_crossings(force_conn))
        end
    end
end

@testset "max_overlaps interaction" begin
    # Crowded layout: 6 long labels packed into a tiny viewport. With a 150×150 px
    # figure and limits spanning 0.4–0.6, the axis-scene is only ~90 px wide while
    # each label measures ~70 px — well over half the viewport. The solver can't
    # spread all six without leaving residual overlaps, so max_overlaps = 2 drops
    # the most crowded ones. We assert: (a) at least one label dropped, (b)
    # connectors for non-dropped labels still have no crossings.
    anchors = Point2f[(0.50, 0.50), (0.51, 0.50), (0.50, 0.51),
                      (0.49, 0.50), (0.50, 0.49), (0.51, 0.51)]
    labels = ["aaaaaaaa", "bbbbbbbb", "cccccccc",
              "dddddddd", "eeeeeeee", "ffffffff"]

    fig = Figure(size = (150, 150))
    ax = Axis(fig[1, 1], limits = (0.4, 0.6, 0.4, 0.6))
    plt = textrepel!(ax, anchors; text = labels, max_overlaps = 2)
    Makie.update_state_before_display!(fig)

    offsets   = plt.attributes[:computed_offsets][]
    anchors_p = plt.attributes[:computed_anchors][]
    sizes_p   = plt.attributes[:computed_sizes][]
    dropped   = plt.attributes[:computed_dropped][]
    params    = plt.attributes[:computed_params][]
    min_len   = plt.min_segment_length[]

    @test count(dropped) ≥ 1   # crowded enough that at least one was dropped
    connectors = [connector_for(anchors_p[i], offsets[i], sizes_p[i], dropped[i], params, min_len)
                  for i in eachindex(offsets)]
    @test isempty(find_crossings(connectors))   # repair pass doesn't corrupt visible labels
end

@testset "recipe solve equals a direct solve_cluster call (#12 structural defense)" begin
    fig = Figure()
    ax  = Axis(fig[1, 1])
    pts = [Point2f(1, 1), Point2f(2, 2), Point2f(1.5, 2.5), Point2f(2.2, 1.1)]
    plt = textrepel!(ax, pts; text = ["alpha", "beta", "gamma", "delta"])
    Makie.update_state_before_display!(fig)

    anchors = plt.attributes[:computed_anchors][]
    sizes   = plt.attributes[:computed_sizes][]
    params  = plt.attributes[:computed_params][]
    direct  = MakieTextRepel.solve_cluster(MakieTextRepel.ProjectionSolver(params),
                                           anchors, sizes, params.bounds)
    @test plt.attributes[:computed_offsets][] == direct.offsets
    @test plt.attributes[:computed_dropped][] == direct.dropped
end

@testset "recipe fans out exactly-coincident anchors (#12, intended byte-identity exception)" begin
    # The coincident golden-angle fan-out in initial_offsets is gated on
    # `cell === nothing && coord_counts > 1`, NOT on pin_mask — so it fires on the
    # recipe path too. For EXACTLY coincident anchors the recipe no longer reproduces
    # the pre-refactor TR-slot collapse; instead both labels are seeded in distinct
    # directions and separate. This is intended (the spec's coincident-separation
    # goal), and this test documents/guards it so the byte-identity story is explicit:
    # byte-identity holds for distinct anchors; exactly-coincident anchors are the
    # one carved-out exception.
    fig = Figure()
    ax  = Axis(fig[1, 1])
    pts = [Point2f(1, 1), Point2f(1, 1), Point2f(2, 2)]   # 1 and 2 exactly coincident
    plt = textrepel!(ax, pts; text = ["a", "b", "c"])
    Makie.update_state_before_display!(fig)

    anchors = plt.attributes[:computed_anchors][]
    offs    = plt.attributes[:computed_offsets][]
    # The two coincident anchors must land at distinct rendered positions (fanned),
    # not collapsed onto the same offset.
    @test anchors[1] .+ offs[1] != anchors[2] .+ offs[2]
end

@testset "markersize attribute + default point_padding (#21)" begin
    using CairoMakie
    xs = [0.0, 1.0, 2.0]; ys = [0.0, 1.0, 0.5]
    labels = ["a", "b", "c"]

    # Default point_padding is now 5.0 on the recipe surface.
    fig = Figure(); ax = Axis(fig[1, 1])
    p = textrepel!(ax, xs, ys; text = labels)
    Makie.update_state_before_display!(fig)
    @test p.computed_params[].point_padding == 5.0

    # markersize sets effective point_padding = m/2 + 0.5.
    fig2 = Figure(); ax2 = Axis(fig2[1, 1])
    p2 = textrepel!(ax2, xs, ys; text = labels, markersize = 9)
    Makie.update_state_before_display!(fig2)
    @test p2.computed_params[].point_padding == 9 / 2 + 0.5   # == 5.0

    # markersize OVERRIDES an explicit point_padding.
    fig3 = Figure(); ax3 = Axis(fig3[1, 1])
    p3 = textrepel!(ax3, xs, ys; text = labels, markersize = 20, point_padding = 99.0)
    Makie.update_state_before_display!(fig3)
    @test p3.computed_params[].point_padding == 20 / 2 + 0.5  # markersize wins

    # A Vector markersize raises a clear error. The validation lives in the solve lift,
    # which this Makie/ComputePipeline version evaluates eagerly at plot-connect time,
    # so the ArgumentError surfaces from the textrepel! call itself.
    fig4 = Figure(); ax4 = Axis(fig4[1, 1])
    @test_throws Exception begin
        p4 = textrepel!(ax4, xs, ys; text = labels, markersize = [9, 9, 9])
        Makie.update_state_before_display!(fig4)
        p4.computed_params[]      # force the lift to run ⇒ ArgumentError propagates
    end
end

@testset "hero dataset geometry: no marker occlusion (#21)" begin
    using CairoMakie
    using Random              # Random.seed!, randn (global RNG)
    using MakieTextRepel: label_cost

    function hero_dataset()
        Random.seed!(20260527)
        knot_centers = [(-0.7, 0.55), (0.75, -0.45), (0.05, 0.9)]
        knot_counts  = [3, 3, 2]
        xs = Float64[]; ys = Float64[]
        for (c, k) in zip(knot_centers, knot_counts), _ in 1:k
            push!(xs, c[1] + 0.03 * randn()); push!(ys, c[2] + 0.03 * randn())
        end
        for _ in 1:14
            push!(xs, 0.85 * randn()); push!(ys, 0.85 * randn())
        end
        labels = ["node $(i)" for i in 1:length(xs)]
        return xs, ys, labels
    end

    xs, ys, labels = hero_dataset()
    # Reproduce the COMMITTED hero artifact geometry (examples/readme_example.jl): a
    # 2100×700 figure with three aspect=1 titled panels; textrepel! lives in the middle.
    # An earlier version of this test rendered a single 400×400 axis — a DIFFERENT pixel
    # viewport that masked extra drops on the real 3-panel artifact (caught in PR review
    # round 1: the committed 1200×400 hero dropped node 11 and node 19). At the hero size
    # the scene has room, so the marker-clearance floor clears every marker (own + foreign)
    # with NO label dropped — the honest "point_overlaps 1→0, no extra drops" result.
    fig = Figure(size = (2100, 700), fontsize = 15)
    Axis(fig[1, 1]; title = "text! (overlapping)", aspect = 1)
    ax2 = Axis(fig[1, 2]; title = "textrepel! (resolved)", aspect = 1)
    p = textrepel!(ax2, xs, ys; text = labels, fontsize = 13, markersize = 9)
    scatter!(ax2, xs, ys; color = :tomato, markersize = 9)
    Axis(fig[1, 3]; title = "annotation! + TextRepelAlgorithm", aspect = 1)
    Makie.update_state_before_display!(fig)

    dropped = p.computed_dropped[]
    q = label_cost(p.computed_anchors[], p.computed_sizes[]; offsets = p.computed_offsets[],
                   bounds = p.computed_params[].bounds, dropped = dropped,
                   box_padding = 4.0,
                   point_padding = 9 / 2 + 0.5,   # markersize=9 ⇒ radius 4.5 + 0.5px gap = 5.0
                   min_segment_length = 2.0)
    @test count(dropped) == 0      # all 22 labels retained (no over-capacity drop)
    @test q.point_overlaps == 0    # every marker cleared (1 → 0 vs pre-#21)
end
