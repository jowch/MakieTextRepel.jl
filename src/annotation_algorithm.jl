# annotation_algorithm.jl — algorithm plug-in for Makie.annotation!

export TextRepelAlgorithm, solve_stats

"""
    TextRepelAlgorithm(; force, force_point, force_pull, only_move,
                         box_padding, point_padding, max_iter,
                         obstacles = Rect2f[], ...)
    TextRepelAlgorithm(params::RepelParams; obstacles = Rect2f[])

An algorithm plug-in for `Makie.annotation!`. Uses MakieTextRepel's
`ProjectionSolver` (side-select → legalize) to place non-overlapping
labels around data points.

Supports per-label pinning: set entries of `textpositions_offset` to
fixed values to lock those labels and let the solver place the rest.
Honors `reset = false` from `annotation!`'s compute graph to warm-start
solves under `advance_optimization!`.

`obstacles` is a `Vector{Rect2f}` in pixel space — the same coordinate
system `annotation!` uses internally. Convert a data-space rectangle
with `Makie.project`.

# Caveats

- Pinning a label so that its data anchor falls strictly inside the
  rendered bbox suppresses that label's connector line. This is
  `annotation!`'s `p2 in offset_bb && return` behavior, not a wrapper
  bug.
- `bounds`/`max_overlaps` misuse warnings fire once per session (Julia's
  standard logger `maxlog=1` contract), so constructing multiple
  algorithm instances with the same mistake yields one warning total.
- `solve_stats` returns the Q diagnostics `(overlaps, point_overlaps, mean_leader,
  crossings, iter, residual, dropped)` of the **most recent** solve.
  A single `TextRepelAlgorithm` instance shared across multiple plots
  reports only the last one to call `calculate_best_offsets!`. Use
  separate instances per plot if you need per-plot diagnostics.

# Scope

`max_overlaps` and background boxes are `textrepel!`-only — they have
no equivalent in `annotation!`'s algorithm contract.

See also: [`textrepel!`](@ref), [`solve_stats`](@ref).
"""
struct TextRepelAlgorithm
    params::RepelParams
    obstacles::Vector{Rect2f}
    solver::ProjectionSolver
end

function TextRepelAlgorithm(; obstacles::Vector{Rect2f} = Rect2f[],
                              bounds = nothing, kwargs...)
    extra = setdiff(keys(kwargs), fieldnames(RepelParams))
    isempty(extra) || throw(ArgumentError(
        "TextRepelAlgorithm: unknown keyword(s) $(collect(extra)). \
         Valid: $(fieldnames(RepelParams)), plus `bounds`, `obstacles`."))

    bounds === nothing || @warn "TextRepelAlgorithm: `bounds` is set \
        automatically from the axis viewport; remove this keyword." maxlog=1

    params = RepelParams(; kwargs...)
    if params.max_overlaps != Inf
        @warn "TextRepelAlgorithm: `max_overlaps` has no equivalent under \
            annotation!; use `textrepel!` if you need label dropping." maxlog=1
    end
    return TextRepelAlgorithm(params, obstacles, ProjectionSolver(params))
end

function TextRepelAlgorithm(params::RepelParams;
                            obstacles::Vector{Rect2f} = Rect2f[])
    if params.max_overlaps != Inf
        @warn "TextRepelAlgorithm: `max_overlaps` has no equivalent under \
            annotation!; use `textrepel!` if you need label dropping." maxlog=1
    end
    return TextRepelAlgorithm(params, obstacles, ProjectionSolver(params))
end

"""
    solve_stats(alg::TextRepelAlgorithm) -> (; overlaps, point_overlaps, mean_leader, crossings, iter, residual, dropped)

Diagnostics from the most recent solve. All zero before any solve runs.
"""
solve_stats(alg::TextRepelAlgorithm) = alg.solver.stats[]

function Makie.calculate_best_offsets!(
        alg::TextRepelAlgorithm,
        offsets::Vector{<:Vec2},
        textpositions::Vector{<:Point2},
        textpositions_offset::Vector{<:Point2},
        text_bbs::Vector{<:Rect2},
        bbox::Rect2;
        maxiter::Union{Makie.Automatic, Int},
        labelspace::Symbol,
        reset::Bool,
    )
    n = length(offsets)
    n == 0 && return
    length(textpositions) == n || throw(DimensionMismatch(
        "textpositions length $(length(textpositions)) does not match offsets length $n"))
    all(p -> all(isfinite, p), textpositions) || throw(ArgumentError(
        "TextRepelAlgorithm: textpositions contains non-finite values"))

    T = eltype(offsets)

    # Per-label pinning detection: finite textpositions_offset[i] entries
    # become pinned offsets in *render-space* (i.e., the displacement
    # annotation! will use when rendering — text_bb.origin + offset).
    pin_mask = BitVector([all(isfinite, p) for p in textpositions_offset])
    pinned_render = Vector{Vec2f}(undef, n)
    for i in 1:n
        if pin_mask[i]
            d = textpositions_offset[i] - textpositions[i]
            pinned_render[i] = Vec2f(d[1], d[2])
        else
            pinned_render[i] = Vec2f(0, 0)
        end
    end

    # All-pinned: bypass solver. Render-space offsets pass through unchanged.
    if all(pin_mask)
        for i in 1:n
            offsets[i] = T(pinned_render[i][1], pinned_render[i][2])
        end
        alg.solver.stats[] = (; overlaps = 0, point_overlaps = 0, mean_leader = 0f0,
                                crossings = 0, iter = 0, residual = 0f0, dropped = 0)
        return
    end

    anchors = [Point2f(p[1], p[2]) for p in textpositions]
    sizes   = [Vec2f(bb.widths[1], bb.widths[2]) for bb in text_bbs]
    annotation_bounds = Rect2f(
        Float32(bbox.origin[1]),  Float32(bbox.origin[2]),
        Float32(bbox.widths[1]),  Float32(bbox.widths[2]),
    )

    # Coordinate translation between solver-space and render-space.
    #
    # The solver places bbox.center at `anchor + solver_offset` (it reasons
    # about box centers). annotation! renders the bbox at
    # `text_bb.origin + render_offset`, whose center is therefore
    # `text_bb.origin + render_offset + widths/2`. Equating the two so the
    # rendered bbox matches what the solver solved for:
    #     render_offset = solver_offset - (text_bb.origin + widths/2 - anchor)
    #                   = solver_offset - align_bias
    # For centered text (text_bb.origin = anchor - widths/2) align_bias is
    # zero; for L/R/T/B-aligned text it is non-zero and must be subtracted
    # from every solver result before writeback, and added to every
    # render-space input before handoff to the solver (including warm-start
    # init_state and pinned obstacle positions).
    bbox_centers = [Point2f(bb.origin[1] + bb.widths[1]/2,
                            bb.origin[2] + bb.widths[2]/2) for bb in text_bbs]
    align_bias = Vec2f[Vec2f(c[1] - a[1], c[2] - a[2])
                       for (c, a) in zip(bbox_centers, anchors)]

    # Pinned offsets: convert render-space → solver-space so the pinned
    # bboxes act as obstacles at their *rendered* position.
    pinned_solver = Vec2f[pin_mask[i] ? pinned_render[i] + align_bias[i] :
                                        Vec2f(0, 0) for i in 1:n]

    # Initial state for the seam:
    # - reset=true: fresh placement → pass `nothing`; solve_cluster does voronoi-init
    #   (Imhof slots, in solver-space) + crossing-repair. No align_bias needed on the
    #   init: the writeback below subtracts align_bias, and rendered box-center =
    #   anchor + solver_offset for any alignment.
    # - reset=false: warm-start from the previous render-space offsets, translated to
    #   solver-space by adding align_bias.
    init_state = reset ? nothing :
                 Vec2f[Vec2f(offsets[i][1], offsets[i][2]) + align_bias[i] for i in 1:n]

    r = solve_cluster(alg.solver, anchors, sizes, annotation_bounds;
                      init_state     = init_state,
                      pin_mask       = pin_mask,
                      pinned_offsets = pinned_solver,
                      obstacles      = alg.obstacles)

    # Writeback: solver-space → render-space (subtract align_bias). Pinned indices
    # recover pinned_render[i] exactly (solve_cluster holds them at pinned_solver[i]).
    for i in 1:n
        o = r.offsets[i] .- align_bias[i]
        if all(isfinite, o)
            offsets[i] = T(o[1], o[2])
        else
            offsets[i] = T(0, 0)
        end
    end
    return
end
