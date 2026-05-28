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
