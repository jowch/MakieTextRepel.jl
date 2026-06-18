# warm_solve.jl — the public stateless warm-start placement primitive.
#
# A thin, render-free face over the internal ProjectionSolver/solve_cluster seam
# (kept internal per issue #8). Built for consumers that re-solve placement on
# every frame (e.g. animated zoom-dives) and want to warm-start from the previous
# frame's offsets — relax, not re-seed — keyed by their own stable ids.

"""
    warm_solve(anchors, sizes, bounds;
               init_state = nothing, pin_mask = nothing,
               pinned_offsets = Vec2f[], obstacles = Rect2f[],
               only_move = :both, box_padding = 4.0,
               point_padding = 5.0, min_segment_length = 2.0)
        -> (; offsets::Vector{Vec2f}, dropped::BitVector, iter::Int, residual::Float32)

Place `length(anchors)` text labels around their `anchors` using the default
`ProjectionSolver` (Voronoi seed → side-select → crossing repair → constraint-
projection legalize → geometric over-capacity drop), returning per-label pixel
offsets and a drop mask. This is the low-level placement primitive that the
`textrepel!` recipe and `TextRepelAlgorithm` use internally, exposed as a stable
public function decoupled from the render/annotation machinery.

All distances are in pixels and all geometry is in one consistent pixel space.

# Arguments
- `anchors::Vector{Point2f}` — the data points, projected to pixel space.
- `sizes::Vector{Vec2f}` — each label's *unpadded* `(width, height)` in px (e.g.
  from `MakieTextRepel.measure_labels`). Same length as `anchors`.
- `bounds::Rect2f` — the clamp region (axis viewport) in the same pixel space.

# Keyword arguments
- `init_state` — `nothing` ⇒ fresh placement (the solver seeds and repairs
  itself). A `Vector{Vec2f}` of per-label offsets ⇒ **warm-start**: relax that
  layout in place (legalize only), which is what keeps per-frame animation smooth.
- `pin_mask` / `pinned_offsets` — `pin_mask[i] == true` holds label `i` fixed at
  `pinned_offsets[i]` while its box still repels the others. Both must have length
  `length(anchors)` when `pin_mask` is given.
- `obstacles::Vector{Rect2f}` — extra keep-out boxes the labels must avoid.
- `only_move` (`:both` | `:x` | `:y`), `box_padding`, `point_padding`,
  `min_segment_length` — the `ProjectionSolver`-relevant `RepelParams` knobs.
  `point_padding` is the marker-clearance gap (default `5.0`, matching the
  `textrepel!` and `TextRepelAlgorithm` surfaces).

# Determinism
Deterministic: identical inputs always produce byte-identical `offsets` and
`dropped` (no RNG — the one randomized dependency, DelaunayTriangulation, is
seeded with `MersenneTwister(0)` and its inputs are lexicographically sorted).
Consumers may golden-test placement directly. (Exactly-coincident anchors fan out
along a fixed golden-angle spiral — still fully reproducible.)

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
    params = RepelParams(; only_move = only_move,
                           box_padding = Float64(box_padding),
                           point_padding = Float64(point_padding),
                           min_segment_length = Float64(min_segment_length))
    return solve_cluster(ProjectionSolver(params), anchors, sizes, bounds;
                         init_state = init_state, pin_mask = pin_mask,
                         pinned_offsets = pinned_offsets, obstacles = obstacles)
end
