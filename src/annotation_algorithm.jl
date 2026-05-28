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

    result = solve_repel(anchors, sizes, effective_params)

    alg.last_iter[] = result.iter
    alg.last_residual[] = result.residual

    for i in 1:n
        offsets[i] = T(result.offsets[i][1], result.offsets[i][2])
    end
    return
end
