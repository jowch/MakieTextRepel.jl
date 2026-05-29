# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

MakieTextRepel.jl is a `ggrepel`/`adjustText`-style label-repel package for [Makie](https://docs.makie.org). It ships two surfaces over the same deterministic solver: `textrepel!` (a standalone Makie recipe with the full feature set — solver, dropping, background boxes, connector trimming) and `TextRepelAlgorithm` (a plug-in for `Makie.annotation!` that reuses the solver underneath `annotation!`'s styling — `Ann.Styles.LineArrow()`, custom paths, arrow heads). Julia 1.11, Makie 0.24.

## Dependency note (important)

This package depends on the **unregistered** TextMeasure.jl (`jowch/TextMeasure.jl`) for render-free text measurement. `Project.toml` `[sources]` currently points at a **local sibling checkout** (`../TextMeasure.jl`), so a clone of that repo must exist beside this one. CI clones it explicitly (see `.github/workflows/CI.yml`). Switching `[sources]` from the relative path to the `{url = "..."}` form is a tracked release-blocker (issue #5) — do not flip it until TextMeasure's `main` is pushed.

## Setup & dev loop

```bash
# First-time setup (needs the sibling ../TextMeasure.jl checkout — see above):
julia --project=. -e 'using Pkg; Pkg.instantiate()'

# Fast iteration without paying Pkg.test() each time. The package env (--project=.)
# has MakieTextRepel + Makie, so you can exercise the pure layers directly in a REPL:
julia --project=. -i -e 'using MakieTextRepel'   # e.g. MakieTextRepel.solve_repel(...)

# Interactive *rendering* needs a Makie backend. CairoMakie is a test-only [extra],
# NOT in [deps] (there is no test/Project.toml), so make a scratch env for it:
julia -e 'using Pkg; Pkg.activate("/tmp/mtr-dev"); Pkg.develop(path="."); Pkg.add("CairoMakie")'
julia --project=/tmp/mtr-dev -i -e 'using CairoMakie, MakieTextRepel'

# Regenerate the README hero image (writes assets/example.png):
julia --project=. examples/readme_example.jl
```

## Testing

Julia precompilation makes each `Pkg.test()` run cost minutes. **Run the suite once, tee it to an agent-scoped log, then `grep` the log** instead of re-running. Pick one stable slug for your session (your agent/job id) and reuse it for both the run and every grep.

```bash
# test/output/ is gitignored, so logs there are never committed.
LOG="test/output/test-<your-agent-id>.log"

# Run ONCE (sets up the test target env: Test + CairoMakie):
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"

# Thereafter, read results from the log — do NOT re-run:
grep -E "Test Summary|Fail|Error|Pass" "$LOG"
grep -nA3 -i "fail\|error" "$LOG"        # locate failures with context
```

The suite has no per-`@testset` CLI filter, so to focus on one area, read the relevant `test/test_*.jl` file and grep its `@testset` names in the log. Re-run the full suite only after you change code.

There is no separate lint/format step configured.

## Architecture

Three pure layers feed a thin Makie recipe. Keep the layers Makie-free where they already are — it's what makes the output deterministic and unit-testable.

- **`src/geometry.jl`** — pure AABB math (GeometryBasics only): `box_at`, `overlap_push`/`point_push` (anisotropic per-axis separation; aligned axes push along the *other* axis so coincident pairs don't run away together), `clip_to_box_edge` (connector attachment; returns `nothing` when the target is strictly inside), `clamp_box_offset` (minimal shift to confine a box to bounds).
- **`src/solver.jl`** — pure deterministic force-directed solver, **no Makie types**. `RepelParams` (all distances in pixels; carries a copy-with-overrides constructor `RepelParams(base; kwargs...)`) + `solve_repel(anchors, sizes, params; obstacles, init_state, pin_mask, pinned_offsets) -> (; offsets, dropped, iter, residual)`. Determinism comes from `init_offsets` (a golden-angle spiral keyed on label index — no RNG). Forces: label↔label repulsion, label↔point repulsion (includes a label's *own* anchor, balanced by an inward `force_pull` spring), user-supplied `obstacles` (`Vector{Rect2f}`) joining the force loop via `overlap_push`, step-cap cooling. `pin_mask`/`pinned_offsets` hold per-label offsets fixed throughout iteration while their boxes still act as obstacles for the rest; pinned offsets bypass `only_move`. **Determinism contract:** the `bounds === nothing` path is byte-identical to the pre-clamping output; the cooling/clamp logic only runs when `bounds` is set. Preserve this when editing the solver.
- **`src/connectors.jl`** — pure leader-line construction → flat `Point2f` pairs for `linesegments!`. Trims the anchor end by `point_padding`, clips the label end to the box face, and suppresses segments that are dropped, anchor-inside-box, or shorter than `min_segment_length`.
- **`src/measure.jl`** — label → `(width, height)` px, all render-free via TextMeasure. Plain strings/LaTeX go through the layout engine (`prepare`/`layout`); rich text (`Makie.RichText`) goes through `measure_bounds` (measured at `px_per_unit=1`, post-scaled by `ppu`); any other label type raises a clear `ArgumentError`. No `Scene` is allocated (issue #6 resolved — the old `Scene`/`full_boundingbox` fallback is gone).
- **`src/recipe.jl`** — the `textrepel!` Makie recipe. `@recipe TextRepel` declares all attributes (the docstrings there are the source of truth for tunables: `force`, `force_point`, `force_pull` as anisotropic `(x,y)` tuples; `only_move`; `max_overlaps`; connector and `background` box attrs). `Makie.plot!` wires the reactive graph: project data anchors → pixel space, `lift` measure+solve on any input change, then render text at **data** positions with per-label **pixel** offsets, plus optional background `poly!` and connector `linesegments!`.
- **`src/annotation_algorithm.jl`** — algorithm plug-in for `Makie.annotation!`. `TextRepelAlgorithm` wraps a `RepelParams` + obstacle list + `Ref`-stored diagnostics (`solve_stats(alg) -> (iter, residual)`), exposing two constructors: kwarg-forwarding (`TextRepelAlgorithm(; force, only_move, ..., obstacles)`, with unknown-kwarg validation and warn-once guards on misused `bounds`/`max_overlaps`) and explicit (`TextRepelAlgorithm(params::RepelParams)`). The registered `Makie.calculate_best_offsets!` method translates `annotation!`'s call shape (text bboxes, viewport, `textpositions_offset`, `reset`) to the solver's: maps finite `textpositions_offset` entries to pinned offsets and NaN entries to auto-placed labels, pre-biases `init_state` with `bbox_center - textposition` + golden-angle perturbation (so non-center alignment and coincident anchors both work), warm-starts from the incoming `offsets` vector when `reset=false`, and feeds `alg.obstacles` through. Adds back `align_bias` in the writeback only on the `reset=true` path. The dispatch hook is undocumented Makie internals; `test/test_annotation_algorithm.jl` carries a stability canary that fails loudly if Makie renames or re-signatures it.

### Two gotchas that have bitten before

1. **Axis-limit leakage.** Text/box/connector children live in pixel space and would inflate the axis limits. The recipe overrides **both** `Makie.data_limits` *and* `Makie.boundingbox` (bottom of `recipe.jl`) so autolimits track only the data anchors — Makie's linear-scale path uses `boundingbox(scene, exclude)`, not `data_limits`, so overriding one alone is insufficient. There's a regression test for this.
2. **ComputeGraph, not a dict.** In Makie 0.24 `p.attributes` is a `ComputeGraph`. Use `Makie.add_input!`, not `setindex!`. Computed offsets are exposed as `p.computed_offsets` for tests/downstream use.

## Project state & history

- Design specs and TDD plans live in `docs/superpowers/specs/` and `docs/superpowers/plans/` — read the relevant one before reworking a subsystem.
- Deferred work is tracked as GitHub issues (#3 glyph-fallback, #4 rounded `cornerradius`, #5 `[sources]` URL flip). (#6 rich-text measurement is resolved — see `src/measure.jl`.) v0.2 (Voronoi init + leader-line crossing repair) is specced/planned but paused pending a solver-API spike branch.
- `examples/readme_example.jl` reproduces the README hero image.
