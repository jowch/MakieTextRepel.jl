# annotation_algorithm.jl — Makie.annotation! algorithm plug-in.

export TextRepelAlgorithm, solve_stats

"""
    TextRepelAlgorithm(; force, force_point, force_pull, only_move,
                         box_padding, point_padding, max_iter,
                         obstacles = Rect2f[], ...)
    TextRepelAlgorithm(params::RepelParams; obstacles = Rect2f[])

`Makie.annotation!` algorithm plug-in. Runs `ProjectionSolver`
(side-select → legalize) for non-overlapping label placement.

Per-label pinning: finite entries in `textpositions_offset` pin those
labels; the solver places the rest. Honors `reset = false` to warm-start
under `advance_optimization!`.

`obstacles`: `Vector{Rect2f}` in pixel space (same as `annotation!`
internals). Convert data-space rectangles with `Makie.project`.

## Key defaults

- `point_padding` defaults to `5.0` px: minimum clearance between each
  anchor and any non-pinned label box. Pass `point_padding = 0.0` to
  disable. A pinned label placed over its own marker is held
  bit-identically — not pushed or dropped.

## Caveats

- Pinning a label with its anchor strictly inside the rendered bbox
  suppresses that label's connector. This is `annotation!`'s
  `p2 in offset_bb && return` behavior, not a wrapper bug.
- `bounds`/`max_overlaps` misuse warnings fire once per session
  (`maxlog=1`); multiple bad-kwarg instances yield one warning total.
- `solve_stats` returns Q diagnostics `(overlaps, point_overlaps, mean_leader,
  crossings, iter, residual, dropped)` for the **most recent** solve.
  An instance shared across plots reports only the last
  `calculate_best_offsets!` call. Use separate instances for per-plot
  diagnostics.

## Scope

`max_overlaps` and background boxes are `textrepel!`-only; no equivalent
in `annotation!`'s algorithm contract.

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

    params = RepelParams(; merge((; point_padding = 5.0), values(kwargs))...)
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

    # Finite textpositions_offset[i] → pinned offset in render-space
    # (displacement annotation! uses: text_bb.origin + offset).
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

    # All-pinned fast path: bypass solver, pass render-space offsets through.
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

    # Solver-space vs render-space:
    # Solver centers the box at anchor + solver_offset.
    # annotation! renders at text_bb.origin + render_offset, center =
    #   text_bb.origin + render_offset + widths/2.
    # Equating:  render_offset = solver_offset - align_bias
    #   where    align_bias = bbox_center - anchor
    # (zero for centered text; non-zero for L/R/T/B alignment).
    # Subtract align_bias on writeback; add it on inputs to the solver
    # (warm init_state, pinned offsets).
    bbox_centers = [Point2f(bb.origin[1] + bb.widths[1]/2,
                            bb.origin[2] + bb.widths[2]/2) for bb in text_bbs]
    align_bias = Vec2f[Vec2f(c[1] - a[1], c[2] - a[2])
                       for (c, a) in zip(bbox_centers, anchors)]

    # Pinned offsets: render-space → solver-space (pinned bboxes act as
    # obstacles at their rendered positions).
    pinned_solver = Vec2f[pin_mask[i] ? pinned_render[i] + align_bias[i] :
                                        Vec2f(0, 0) for i in 1:n]

    # reset=true  → fresh placement (pass nothing; solve_cluster runs voronoi-init +
    #   crossing repair in solver-space; no align_bias needed on init).
    # reset=false → warm-start from previous render-space offsets, translated to
    #   solver-space by adding align_bias.
    init_state = reset ? nothing :
                 Vec2f[Vec2f(offsets[i][1], offsets[i][2]) + align_bias[i] for i in 1:n]

    r = solve_cluster(alg.solver, anchors, sizes, annotation_bounds;
                      init_state     = init_state,
                      pin_mask       = pin_mask,
                      pinned_offsets = pinned_solver,
                      obstacles      = alg.obstacles)

    # Writeback: solver-space → render-space (subtract align_bias).
    # Pinned labels recover pinned_render[i] exactly (held at pinned_solver[i]).
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
