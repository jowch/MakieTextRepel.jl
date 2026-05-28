# Changelog

All notable changes to MakieTextRepel.jl are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
- README with a `text!`-vs-`textrepel!` hero image, a runnable example
  (`examples/readme_example.jl`), a visual smoke test, and GitHub Actions CI.

### Changed

- Default `point_padding` bumped `0 → 2` px and `min_segment_length` `5 → 2` px
  alongside own-point repulsion, so connectors leave a visible gap at the marker.

### Fixed

- Axis-limit blow-up: the recipe now overrides both `Makie.data_limits` and
  `Makie.boundingbox` so axis autolimits track only the data anchors and are not
  inflated by the pixel-space text, box, and connector children.

[Unreleased]: https://github.com/jowch/MakieTextRepel.jl/commits/main
