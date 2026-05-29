# Render-free rich-text measurement: design spec

Date: 2026-05-28
Status: Draft v1
Branch: `issue-6-rich-text-measurement`
Closes: #6

## Goal

Eliminate the `Scene`/`text!`/`full_boundingbox` fallback in `src/measure.jl`
by routing `Makie.RichText` labels through TextMeasure's render-free
`measure_bounds`. Plain-string measurement is untouched. The user-visible
behavior of `textrepel!` (and `TextRepelAlgorithm`) is unchanged; only the
measurement mechanism for rich text changes — from a per-label mini-render to
arithmetic over font glyph advances.

## Project context

- `measure.jl` turns each label into a `(width, height)` pixel box that feeds
  the solver's label↔label repulsion. It is on the reactive hot path:
  `recipe.jl:80` calls `measure_labels` inside a `lift`, so every window
  **resize** and parameter change re-measures every label.
- Today there are two `measure_one` methods:
  - `measure_one(::AbstractString, …)` — `TextMeasure.prepare` → `layout`,
    render-free. Covers `String` and `LaTeXString`.
  - `measure_one(label, …)` — catch-all fallback for `Makie.RichText` (and any
    non-string). Allocates a throwaway `Scene`, calls `text!`,
    `update_state_before_display!`, then reads `full_boundingbox(t, :pixel)`
    and scales by `ppu`. Correct, but allocates a `Scene` and runs Makie's
    layout machinery per label, every resize.
- Issue #6 was blocked on `jowch/TextMeasure.jl#1` (TextMeasure lacked
  render-free rich-text measurement). **That blocker is resolved**:
  `TextMeasure.measure_bounds(::MakieBackend, ::Makie.RichText) -> TextBounds`
  is on TextMeasure's **pushed `origin/main`**
  (`ext/TextMeasureMakieExt.jl:163`), tested in `test/test_richtext.jl`, and
  built to match Makie's `text!` output exactly at `px_per_unit = 1`.
- `Project.toml [sources]` points at the sibling `../TextMeasure.jl` checkout,
  which is on `main` and already carries the API. **This work needs no
  `[sources]` change** (issue #5 stays independent).

## Decisions

### D1 — Keep two methods; swap only the rich one

The issue's "collapse into a single dispatch" is aspirational: `measure_bounds`
dispatches **only** on `Makie.RichText`, not `AbstractString`. The two tracks
reflect two genuinely different TextMeasure subsystems — a flat layout engine
(`prepare`/`layout`) for strings and a tree-walking bbox measurer
(`measure_bounds`) for styled RichText. We therefore keep the string method
as-is and replace the *rich* method's internals:

```julia
# Plain strings (and LaTeXString) → TextMeasure layout engine. UNCHANGED.
function measure_one(label::AbstractString, font, fontsize::Float64, ppu::Float64)
    f = _resolve_font(font)
    backend = TextMeasure.MakieBackend(; font = f, fontsize = fontsize, px_per_unit = ppu)
    lay = TextMeasure.layout(TextMeasure.prepare(backend, String(label)))
    return Vec2f(lay.size[1], lay.size[2])
end

# Rich text → TextMeasure measure_bounds, render-free. NEW (replaces the Scene fallback).
function measure_one(label::Makie.RichText, font, fontsize::Float64, ppu::Float64)
    f = _resolve_font(font)
    backend = TextMeasure.MakieBackend(; font = f, fontsize = fontsize, px_per_unit = 1.0)
    tb = TextMeasure.measure_bounds(backend, label)
    return Vec2f(tb.size[1] * ppu, tb.size[2] * ppu)
end
```

The catch-all `measure_one(label, …)` Scene method is **deleted**, along with
any now-unused `Scene`/`text!`/`full_boundingbox` references that exist only
for it (verify they are not used elsewhere before removing imports).

### D2 — `px_per_unit` handled at our boundary

`measure_bounds` hard-requires `px_per_unit == 1.0` and throws otherwise. The
reason is real, not caution: its multi-line line-drop is a hardcoded 20px stub
(Makie's `apply_lineheight!`) that does not scale with ppu, so measuring
multi-line RichText at ppu≠1 would skew heights
(`ext/TextMeasureMakieExt.jl:165-168`).

We **absorb** this restriction: always build the backend at `px_per_unit = 1.0`
and apply `ppu` ourselves as a post-scale (`tb.size .* ppu`). This is
byte-identical in shape to what the old fallback did
(`full_boundingbox(:pixel) * ppu`, where `:pixel` is logical pixels). Net
effect: the raw `ArgumentError` from TextMeasure is **never surfaced to any
caller**, including a direct caller passing `ppu ≠ 1`.

`px_per_unit` is a *rasterization* concern, not a *layout* concern. The recipe
hardcodes `measure_labels(…, 1.0)` (`recipe.jl:80`) and solves entirely in
logical pixels; a user exporting at `save(...; px_per_unit = 2)` is rasterized
by Makie at save time and **never drives `measure_bounds` at ppu≠1**. So the
plotting path cannot hit the guard.

### D3 — Acceptance via a golden test, not a production fallback

We do not keep a defensive Scene fallback (that would violate the AC and leave
dead-in-practice code). Instead we prove equivalence in the test layer: render
the old way *inside the test only* and compare.

## Behavior changes

- An exotic label type that is neither `AbstractString` nor `Makie.RichText`
  now raises `MethodError` instead of silently rendering. Acceptable: Makie's
  `text` accepts only `String` / `LaTeXString` / `RichText`.

## Known limitations (carried, not introduced)

- **Multi-line rich text at `ppu ≠ 1`**: the 20px line-drop is scaled
  externally by `ppu`, which may diverge sub-pixel from a true Makie render at
  that ppu. This matches today's fallback behavior exactly, is unreachable from
  the recipe (always ppu=1), and is an upstream TextMeasure refinement (make
  the line-drop scale, relax the guard) if it ever matters. Not in scope here.
- **Glyph fallback** (`measure.jl:13` TODO): routing through `text_bb` when the
  resolved font lacks a label's glyphs is issue #3. Out of scope.

## Testing

`test/test_measure.jl` — upgrade the rich-text case from "finite & positive" to
a **golden comparison**:

- For a representative set of RichText constructs — simple `rich("a", "b")`,
  subscript (`rich("H", subscript("2"), "O")`), superscript, mixed font/size,
  and a multi-line example — compute:
  1. the new `measure_one(label, …)` (via `measure_labels`), and
  2. a one-off Scene + `text!` + `full_boundingbox(t, :pixel)` measurement
     (the old algorithm, lives only in the test).
- Assert the two agree within a sub-pixel tolerance (`isapprox` with a small
  `atol`, e.g. ≤ 1px). This is the issue's acceptance criterion
  ("float-identical / sub-pixel to today's render-based numbers").
- Keep the existing plain-string assertions unchanged.
- After removal, no production code references `Scene` for measurement; the
  "no `Scene` allocation in `measure_labels`" AC is met structurally.

Run the suite once per the CLAUDE.md test loop (tee to an agent-scoped log under
`test/output/`, then grep), since `Pkg.test()` pays precompilation each run.

## Acceptance criteria (from #6)

- [ ] Rich-text labels measured render-free — no `Scene` allocation in
  `measure_labels`.
- [ ] New measurements float-identical (or sub-pixel) to today's render-based
  numbers on a representative rich-text set (golden test).
- [ ] `textrepel!` rich-text behavior unchanged for users.

## Out of scope

- `[sources]` URL flip (#5), glyph fallback (#3), `prepare`/`layout` ↔
  `measure_bounds` unification (would require an upstream
  `measure_bounds(::MakieBackend, ::AbstractString)` — a legitimate future
  cleanup, deliberately deferred).

## Implementation gotcha

Per project memory: running `Pkg.resolve()` inside a `.claude/worktrees/`
checkout rewrites the TextMeasure `[sources]` path away from the sibling
(caught in PR #11). If implementing in a worktree, use the symlink workaround.
