# annotation_algorithm.jl — algorithm plug-in for Makie.annotation!

export TextRepelAlgorithm, solve_stats

"""
    TextRepelAlgorithm(; force, force_point, force_pull, only_move,
                         box_padding, point_padding, max_iter,
                         obstacles = Rect2f[], ...)
    TextRepelAlgorithm(params::RepelParams; obstacles = Rect2f[])

An algorithm plug-in for `Makie.annotation!`. Uses MakieTextRepel's
force-directed solver to place non-overlapping labels around data points.

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

# Scope

`max_overlaps` and background boxes are `textrepel!`-only — they have
no equivalent in `annotation!`'s algorithm contract.

See also: [`textrepel!`](@ref), [`solve_stats`](@ref).
"""
struct TextRepelAlgorithm
    params::RepelParams
    obstacles::Vector{Rect2f}
    last_iter::Base.RefValue{Int}
    last_residual::Base.RefValue{Float32}
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
    if params.max_overlaps !== Inf
        @warn "TextRepelAlgorithm: `max_overlaps` has no equivalent under \
            annotation!; use `textrepel!` if you need label dropping." maxlog=1
    end
    return TextRepelAlgorithm(params, obstacles, Ref(0), Ref(0f0))
end

function TextRepelAlgorithm(params::RepelParams;
                            obstacles::Vector{Rect2f} = Rect2f[])
    if params.max_overlaps !== Inf
        @warn "TextRepelAlgorithm: `max_overlaps` has no equivalent under \
            annotation!; use `textrepel!` if you need label dropping." maxlog=1
    end
    return TextRepelAlgorithm(params, obstacles, Ref(0), Ref(0f0))
end

"""
    solve_stats(alg::TextRepelAlgorithm) -> (; iter, residual)

Return iteration count and final residual from the most recent solve.
Returns `(iter = 0, residual = 0f0)` before any solve runs.
"""
solve_stats(alg::TextRepelAlgorithm) =
    (iter = alg.last_iter[], residual = alg.last_residual[])

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

    # Per-label pinning detection (full implementation in Task 10).
    pin_mask = BitVector([all(isfinite, p) for p in textpositions_offset])
    pinned_offsets = Vector{Vec2f}(undef, n)
    for i in 1:n
        if pin_mask[i]
            d = textpositions_offset[i] - textpositions[i]
            pinned_offsets[i] = Vec2f(d[1], d[2])
        else
            pinned_offsets[i] = Vec2f(0, 0)
        end
    end

    # All-pinned: bypass solver.
    if all(pin_mask)
        for i in 1:n
            offsets[i] = T(pinned_offsets[i][1], pinned_offsets[i][2])
        end
        alg.last_iter[] = 0
        alg.last_residual[] = 0f0
        return
    end

    anchors = [Point2f(p[1], p[2]) for p in textpositions]
    sizes   = [Vec2f(bb.widths[1], bb.widths[2]) for bb in text_bbs]
    annotation_bounds = Rect2f(
        Float32(bbox.origin[1]),  Float32(bbox.origin[2]),
        Float32(bbox.widths[1]),  Float32(bbox.widths[2]),
    )

    mi = maxiter === Makie.automatic ? alg.params.max_iter : Int(maxiter)
    effective_params = RepelParams(alg.params;
        bounds   = annotation_bounds,
        max_iter = mi,
    )

    # Initial state for the solver:
    # - reset=true (fresh): pre-bias to bbox_center so own-anchor repulsion
    #   pushes from the data point. Returned offsets get align_bias added back
    #   so annotation! receives a displacement reflecting the bbox alignment.
    # - reset=false (warm-start): incoming offsets are textposition-relative
    #   from the previous solve (annotation! writes back whatever we put in
    #   `offsets`, so warm-starting needs no coordinate change). They go in
    #   directly and come out directly.
    if reset
        bbox_centers = [Point2f(bb.origin[1] + bb.widths[1]/2,
                                bb.origin[2] + bb.widths[2]/2) for bb in text_bbs]
        align_bias = Vec2f[Vec2f(c[1] - a[1], c[2] - a[2])
                           for (c, a) in zip(bbox_centers, anchors)]
        init_state = align_bias
    else
        init_state = Vec2f[Vec2f(o[1], o[2]) for o in offsets]
    end

    result = solve_repel(anchors, sizes, effective_params;
                         obstacles      = alg.obstacles,
                         init_state     = init_state,
                         pin_mask       = pin_mask,
                         pinned_offsets = pinned_offsets)

    alg.last_iter[] = result.iter
    alg.last_residual[] = result.residual

    for i in 1:n
        if pin_mask[i]
            offsets[i] = T(pinned_offsets[i][1], pinned_offsets[i][2])
        elseif reset
            o = result.offsets[i] .+ align_bias[i]
            if all(isfinite, o)
                offsets[i] = T(o[1], o[2])
            else
                # Solver produced NaN/Inf — fall back to zero offset for this label.
                offsets[i] = T(0, 0)
            end
        else
            if all(isfinite, result.offsets[i])
                offsets[i] = T(result.offsets[i][1], result.offsets[i][2])
            else
                offsets[i] = T(0, 0)
            end
        end
    end
    return
end
