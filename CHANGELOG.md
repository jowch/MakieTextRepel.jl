# Changelog

All notable changes to MakieTextRepel.jl are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.0] - 2026-05-28

Layouts are now initialized using Imhof-preferred slots within each anchor's Voronoi cell when geometry allows, and a post-solve repair pass eliminates leader-line crossings on the typical inputs we've measured. Existing user code runs unchanged; output positions will differ from v0.1.

### Added

- Voronoi-informed initialization (`src/voronoi.jl`, `src/init.jl`) — labels start at the highest-preference Imhof slot (TR > R > T > BR > L > BL > B > TL) that fits inside their anchor's Voronoi cell, with TR fallback when none fit.
- Crossing repair (`src/crossings.jl`) — deterministic 2-opt position-swap pass that converges to a crossing-free layout in 1–3 outer iterations on the integration fixtures (sparse, dense, mixed) and is bounded by `max_iter = 100`. The non-crossing property is best-effort with a backstop: `repair_crossings!` emits a `@warn` on the rare cap-out case where residual crossings remain after the final scan, so silent degradation can't slip through.
- Internal `AbstractClusterSolver` interface (`src/solvers/`) — seam for future Julia-native solver implementations.
- New dependency: `DelaunayTriangulation.jl` (≥ 1.6). The Voronoi step wraps DT.jl in a `try`/`catch` so any DT failure on degenerate inputs (near-collinear points, etc.) degrades to all-TR Imhof placement rather than crashing the compute graph.

### Changed

- `textrepel!` recipe pipeline now Voronoi-initializes label positions and runs a post-solve crossing-repair pass. Output offsets differ from v0.1 even for identical inputs.

### Notes

- `solve_repel`'s `init_state` kwarg (added in 0.1.0 via the annotation-algorithm spike) is now the channel through which the recipe injects Imhof-derived initial positions.
- `init_offsets` and `_GOLDEN_ANGLE` (`src/solver.jl`) are retained: they back the default branch of `solve_repel` and are called by `TextRepelAlgorithm` for warm-starts.
- The non-crossing property scopes to `textrepel!` only; `TextRepelAlgorithm` (the `Makie.annotation!` plug-in) does not get the repair pass in v0.2.

## [0.1.0] - 2026-05-28

Initial `0.1.0` feature set. Nothing has been tagged or registered yet; everything
below is pending the first release (see issue #5 — the `[sources]` URL flip — which
gates distribution).

### Added

- `textrepel`/`textrepel!` — a `ggrepel`/`adjustText`-style label-repel recipe for
  Makie that displaces overlapping text labels and draws connector lines back to
  their data points. Accepts either a `Vector{Point2f}` or `(xs, ys)`.
- Deterministic force-directed solver (`solve_repel`) in pixel space: golden-angle
  spiral initialization (no RNG), anisotropic label↔label and label↔point
  repulsion, inward `force_pull` spring, and per-iteration step clamping. Same data
  + same figure always yields the same layout.
- Pure axis-aligned bounding-box geometry primitives (overlap/point push, box-edge
  clipping, viewport clamping), independent of Makie.
- Render-free text measurement via the (unregistered) TextMeasure.jl, with a
  Makie `full_boundingbox` fallback for rich text.
- Connector leader-line segments rendered in pixel space, with anchor-end trimming
  (`point_padding`), label-edge clipping, and a `min_segment_length` visibility filter.
- Own-point repulsion: labels are pushed off their own anchor, so isolated labels
  settle beside their point rather than on top of it (issue #1).
- Overlap-count label dropping via `max_overlaps` (`Inf` keeps all labels).
- Optional filled background boxes behind labels (`background`, `backgroundcolor`,
  `strokecolor`, `strokewidth`).
- Axis-viewport clamping (on by default, ggrepel/adjustText style): labels are
  confined to the axis data area and the clamp respects the `only_move` axis lock.
- Tunable attributes: `force`, `force_point`, `force_pull` (anisotropic `(x, y)`
  tuples), `only_move` (`:both`/`:x`/`:y`), `max_iter`, `box_padding`,
  `point_padding`, plus connector and background styling.
- `TextRepelAlgorithm` — algorithm plug-in for `Makie.annotation!` that reuses
  the same force-directed solver underneath `annotation!`'s styling
  (`Ann.Styles.LineArrow()`, custom paths, arrow heads). Constructed via
  keyword forwarding (`TextRepelAlgorithm(; force, only_move, ..., obstacles)`)
  or from an explicit `RepelParams`. Supports per-label pinning (mix finite and
  `NaN` entries in `textpositions_offset` — finite ones become pinned offsets,
  `NaN` entries auto-place around them), warm-start under `advance_optimization!`
  (`reset=false`), and obstacle avoidance via an `obstacles::Vector{Rect2f}`
  keyword. `solve_stats(alg)` returns `(iter, residual)` diagnostics from the
  most recent solve. README has a "Two surfaces" section comparing it to
  `textrepel!`.
- `solve_repel` extended with `obstacles`, `init_state`, `pin_mask`,
  `pinned_offsets` keyword arguments (all optional, defaults preserve prior
  behavior). Returns a NamedTuple `(; offsets, dropped, iter, residual)` so
  callers can introspect convergence; existing positional-destructure call
  sites (`a, b = solve_repel(...)`) still work because NamedTuples support
  positional iteration.
- `RepelParams(base::RepelParams; kwargs...)` copy-with-overrides constructor.
- README with a `text!`-vs-`textrepel!` hero image, a runnable example
  (`examples/readme_example.jl`), a visual smoke test, and GitHub Actions CI.

### Changed

- Default `point_padding` bumped `0 → 2` px and `min_segment_length` `5 → 2` px
  alongside own-point repulsion, so connectors leave a visible gap at the marker.

### Fixed

- Axis-limit blow-up: the recipe now overrides both `Makie.data_limits` and
  `Makie.boundingbox` so axis autolimits track only the data anchors and are not
  inflated by the pixel-space text, box, and connector children.

[0.2.0]: https://github.com/jowch/MakieTextRepel.jl/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/jowch/MakieTextRepel.jl/releases/tag/v0.1.0
