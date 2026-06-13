# Changelog

All notable changes to MakieTextRepel.jl are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

Nothing has been tagged, released, or registered yet — everything below is the
pending first release (distribution is gated on issue #5, the `[sources]` URL
flip). The internal `Project.toml` version bumps during development tracked
milestones only; because no version was ever published, there are **no
user-facing breaking changes** to report.

### Added

#### Recipe and solver

- `textrepel` / `textrepel!` — a `ggrepel`/`adjustText`-style label-repel recipe
  for Makie that displaces overlapping text labels and draws connector lines back
  to their data points. Accepts either a `Vector{Point2f}` or `(xs, ys)`.
- **`ProjectionSolver` — the default placement solver.** Discrete Imhof
  side-selection → crossing repair → Dykstra constraint-projection legalization,
  with a geometric over-capacity drop pass. Guarantees **zero box overlap** on
  feasible scenes and produces short leader lines; fully deterministic and
  RNG-free. Over-capacity scenes deterministically drop the most-overlapped
  labels until the remainder fits overlap-free. Composes three pure
  (GeometryBasics-only) layers: `src/side_select.jl` (greedy discrete Imhof-slot
  refinement), `src/legalize.jl` (Dykstra constraint-projection overlap removal),
  and `src/cost.jl` (`label_cost`, a read-only quality functional reporting
  overlaps / mean leader length / crossings).
- **In-tree force-directed solver** (`solve_repel`, `ForceSolver`) — deterministic
  golden-angle-spiral placement in pixel space (no RNG), anisotropic label↔label
  and label↔point repulsion, inward `force_pull` spring, own-point repulsion (so
  isolated labels settle beside their point, not on it), and per-iteration step
  clamping. Retained as the non-default fallback behind the `AbstractClusterSolver`
  seam.
- `AbstractClusterSolver` interface (`src/solvers/`) — the seam that lets both
  surfaces swap placement strategy; `ProjectionSolver` and `ForceSolver` implement
  it.
- Voronoi-informed initialization (`src/voronoi.jl`, `src/init.jl`) — labels seed
  at the highest-preference Imhof slot (TR > R > T > BR > L > BL > B > TL) that fits
  inside their anchor's Voronoi cell, with TR fallback when none fit. Wraps
  DelaunayTriangulation.jl (≥ 1.6) in a `try`/`catch` plus a collinear guard that
  degrades to all-TR placement on degenerate inputs rather than crashing the
  compute graph.
- Crossing repair (`src/crossings.jl`) — deterministic 2-opt position-swap pass,
  bounded by `max_iter = 100`, `@warn` on cap-out. Under the default solver this is
  **best-effort**: it runs before the final legalize, which can re-introduce a
  crossing (crossings remain no worse than the in-tree `ForceSolver` on the
  fixtures).
- **Marker-clearance floor — point-aware legalization (#21).** The default solver now
  treats every data anchor as a fixed keep-out of radius `point_padding`, so a label's
  text never sits under its own or a neighbour's scatter marker — enforced *after*
  legalization (not just at side-selection, which an earlier legalize pass could undo).
  Implemented as a `soft` node class in `legalize` (keep-outs push labels but are
  excluded from the over-capacity drop decision), so **marker clearance never drops a
  label**. `point_padding` is now the honest marker-clearance knob (default **5 px** on
  the `textrepel!` and `TextRepelAlgorithm` surfaces — clears Makie's default
  `markersize = 9`; the `RepelParams` primitive stays **0 px**, where it doubles as the
  in-tree force solver's point-repulsion halo).
- `markersize` (recipe convenience attribute) — declare your sibling `scatter!` marker
  size and `point_padding` is derived as `markersize/2 + 0.5`. `textrepel!` draws no
  markers itself; this only tells the solver how much to clear (assumes a disc marker
  in `markerspace = :pixel`; set `point_padding` directly for other markers).

#### Annotation plug-in

- `TextRepelAlgorithm` — algorithm plug-in for `Makie.annotation!` that reuses the
  solver underneath `annotation!`'s styling (`Ann.Styles.LineArrow()`, custom
  paths, arrow heads). Constructed via keyword forwarding
  (`TextRepelAlgorithm(; force, only_move, …, obstacles)`) or from an explicit
  `RepelParams`. Supports per-label pinning (mix finite and `NaN` entries in
  `textpositions_offset` — finite ones become pinned offsets, `NaN` entries
  auto-place around them), warm-start under `advance_optimization!` (`reset=false`),
  and obstacle avoidance via `obstacles::Vector{Rect2f}`. `solve_stats(alg)` returns
  the read-only quality functional `(; iter, residual, overlaps, point_overlaps,
  mean_leader, crossings, dropped)` from the most recent solve.

#### Geometry, measurement, connectors, styling

- Pure axis-aligned bounding-box primitives (overlap/point push, box-edge clipping,
  viewport clamping), independent of Makie.
- Render-free text measurement via the (unregistered) TextMeasure.jl.
- Connector leader-line segments in pixel space, with anchor-end trimming
  (`point_padding`), label-edge clipping, and a `min_segment_length` visibility
  filter.
- Optional filled background boxes behind labels (`background`, `backgroundcolor`,
  `strokecolor`, `strokewidth`).
- Axis-viewport clamping (on by default, ggrepel/adjustText style): labels are
  confined to the axis data area, respecting the `only_move` axis lock. The recipe
  overrides both `Makie.data_limits` and `Makie.boundingbox` so axis autolimits
  track only the data anchors and are not inflated by the pixel-space text, box,
  and connector children.
- Tunable attributes: `force`, `force_point`, `force_pull` (anisotropic `(x, y)`
  tuples), `only_move` (`:both`/`:x`/`:y`), `max_iter`, `max_overlaps`,
  `box_padding` (default 4 px), `point_padding` (marker clearance; default 5 px on the
  `textrepel!`/`TextRepelAlgorithm` surfaces, 0 px on the `RepelParams` primitive),
  `markersize` (recipe convenience; derives `point_padding`),
  `min_segment_length` (default 2 px), plus connector and background styling.
  **Note:** `force`, `force_point`, `force_pull`, `max_iter`, and `max_overlaps`
  are inert under the default `ProjectionSolver` (it has no force loop and
  guarantees zero overlap); they affect only the in-tree `ForceSolver`.
- `RepelParams` and its `RepelParams(base::RepelParams; kwargs...)`
  copy-with-overrides constructor; `solve_repel` returns a NamedTuple
  `(; offsets, dropped, iter, residual)` (positional destructuring still works).
- README with a `text!`-vs-`textrepel!` hero image, a runnable example
  (`examples/readme_example.jl`), a visual smoke test, and GitHub Actions CI.
