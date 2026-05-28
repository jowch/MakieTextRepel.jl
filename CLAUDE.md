# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

MakieTextRepel.jl is a `ggrepel`/`adjustText`-style label-repel **recipe** for [Makie](https://docs.makie.org): `textrepel!` displaces overlapping text labels and draws connector lines back to their data points. Julia 1.11, Makie 0.24.

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
- **`src/solver.jl`** — pure deterministic force-directed solver, **no Makie types**. `RepelParams` (all distances in pixels) + `solve_repel(anchors, sizes, params) -> (offsets, dropped)`. Determinism comes from `init_offsets` (a golden-angle spiral keyed on label index — no RNG). Forces: label↔label repulsion, label↔point repulsion (includes a label's *own* anchor, balanced by an inward `force_pull` spring), step-cap cooling. **Determinism contract:** the `bounds === nothing` path is byte-identical to the pre-clamping output; the cooling/clamp logic only runs when `bounds` is set. Preserve this when editing the solver.
- **`src/connectors.jl`** — pure leader-line construction → flat `Point2f` pairs for `linesegments!`. Trims the anchor end by `point_padding`, clips the label end to the box face, and suppresses segments that are dropped, anchor-inside-box, or shorter than `min_segment_length`.
- **`src/measure.jl`** — label → `(width, height)` px. Plain strings/LaTeX go through TextMeasure (render-free); rich text falls back to a throwaway `Scene` + `full_boundingbox`. (Collapsing the fallback is issue #6, blocked upstream.)
- **`src/recipe.jl`** — the only Makie-aware file. `@recipe TextRepel` declares all attributes (the docstrings there are the source of truth for tunables: `force`, `force_point`, `force_pull` as anisotropic `(x,y)` tuples; `only_move`; `max_overlaps`; connector and `background` box attrs). `Makie.plot!` wires the reactive graph: project data anchors → pixel space, `lift` measure+solve on any input change, then render text at **data** positions with per-label **pixel** offsets, plus optional background `poly!` and connector `linesegments!`.

### Two gotchas that have bitten before

1. **Axis-limit leakage.** Text/box/connector children live in pixel space and would inflate the axis limits. The recipe overrides **both** `Makie.data_limits` *and* `Makie.boundingbox` (bottom of `recipe.jl`) so autolimits track only the data anchors — Makie's linear-scale path uses `boundingbox(scene, exclude)`, not `data_limits`, so overriding one alone is insufficient. There's a regression test for this.
2. **ComputeGraph, not a dict.** In Makie 0.24 `p.attributes` is a `ComputeGraph`. Use `Makie.add_input!`, not `setindex!`. Computed offsets are exposed as `p.computed_offsets` for tests/downstream use.

## Project state & history

- Design specs and TDD plans live in `docs/superpowers/specs/` and `docs/superpowers/plans/` — read the relevant one before reworking a subsystem.
- Deferred work is tracked as GitHub issues (#3 glyph-fallback, #4 rounded `cornerradius`, #5 `[sources]` URL flip, #6 rich-text measurement). v0.2 (Voronoi init + leader-line crossing repair) is specced/planned but paused pending a solver-API spike branch.
- `examples/readme_example.jl` reproduces the README hero image.
