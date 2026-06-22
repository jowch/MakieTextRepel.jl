# warm_solve.jl — public stateless warm-start placement primitive.
# Render-free face over the internal ProjectionSolver/solve_cluster seam (issue #8).

"""
    warm_solve(anchors, sizes, bounds;
               init_state = nothing, pin_mask = nothing,
               pinned_offsets = Vec2f[], obstacles = Rect2f[],
               only_move = :both, box_padding = 4.0,
               point_padding = 5.0, min_segment_length = 2.0)
        -> (; offsets::Vector{Vec2f}, dropped::BitVector, iter::Int, residual::Float32)

Place `length(anchors)` labels using the default `ProjectionSolver` (Voronoi seed
→ side-select → crossing repair → constraint-projection legalize → geometric
over-capacity drop). Returns per-label pixel offsets and a drop mask. Decoupled
from the render/annotation machinery; a stable public peer over the same
`solve_cluster` seam that `textrepel!` and `TextRepelAlgorithm` call internally.

All distances in pixels; all geometry in a single consistent pixel space.

# Arguments
- `anchors::Vector{Point2f}` — data points projected to pixel space.
- `sizes::Vector{Vec2f}` — each label's *unpadded* `(width, height)` in px (e.g.
  from `measure_labels`). Same length as `anchors`.
- `bounds::Rect2f` — clamp region (axis viewport) in the same pixel space.

# Keyword arguments
- `init_state` — `nothing` ⇒ fresh placement. A `Vector{Vec2f}` of per-label
  offsets ⇒ **warm-start**: relax in place (legalize only), keeping per-frame
  animation smooth.
- `pin_mask` / `pinned_offsets` — `pin_mask[i] == true` holds label `i` fixed at
  `pinned_offsets[i]`; its box still repels others. Both must have length
  `length(anchors)` when `pin_mask` is given.
- `obstacles::Vector{Rect2f}` — extra keep-out boxes labels must avoid.
- `only_move` (`:both` | `:x` | `:y`), `box_padding`, `point_padding`,
  `min_segment_length` — `RepelParams` knobs passed to the `ProjectionSolver`.
  `point_padding` is the marker-clearance gap (default `5.0`, matching
  `textrepel!` and `TextRepelAlgorithm`).

# Determinism
Identical inputs produce byte-identical `offsets` and `dropped` (no RNG —
DelaunayTriangulation is seeded with `MersenneTwister(0)` and inputs are
lexicographically sorted). Coincident anchors fan out on a fixed golden-angle
spiral. Consumers may golden-test placement directly.

See also [`textrepel!`](@ref), [`TextRepelAlgorithm`](@ref).
"""
function warm_solve(anchors::Vector{Point2f}, sizes::Vector{Vec2f}, bounds::Rect2f;
                    init_state::Union{Nothing,Vector{Vec2f}} = nothing,
                    pin_mask::Union{Nothing,BitVector}       = nothing,
                    pinned_offsets::Vector{Vec2f}            = Vec2f[],
                    obstacles::Vector{Rect2f}                = Rect2f[],
                    only_move::Symbol         = :both,
                    box_padding::Real         = 4.0,
                    point_padding::Real       = 5.0,
                    min_segment_length::Real  = 2.0)
    # Catch sizes/anchors mismatch here; solve_cluster guards pin_mask/init_state but not sizes.
    length(sizes) == length(anchors) || throw(DimensionMismatch(
        "sizes length $(length(sizes)) does not match anchors length $(length(anchors))"))
    params = RepelParams(; only_move = only_move,
                           box_padding = Float64(box_padding),
                           point_padding = Float64(point_padding),
                           min_segment_length = Float64(min_segment_length))
    return solve_cluster(ProjectionSolver(params), anchors, sizes, bounds;
                         init_state = init_state, pin_mask = pin_mask,
                         pinned_offsets = pinned_offsets, obstacles = obstacles)
end
