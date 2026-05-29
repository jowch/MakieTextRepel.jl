# Render-free rich-text measurement: design spec

Date: 2026-05-28
Status: Draft v3 (post spec-review, 2 rounds)
Branch: `issue-6-rich-text-measurement`
Closes: #6

## Goal

Eliminate the `Scene`/`text!`/`full_boundingbox` fallback in `src/measure.jl`
by routing `Makie.RichText` labels through TextMeasure's render-free
`measure_bounds`. Plain-string measurement is untouched. The user-visible
behavior of `textrepel!` (and `TextRepelAlgorithm`) is unchanged for the
supported label types; only the measurement mechanism for rich text changes —
from a per-label mini-render to arithmetic over font glyph advances.

## Project context

- `measure.jl` turns each label into a `(width, height)` pixel box that feeds
  the solver's label↔label repulsion. It is on the reactive hot path:
  `recipe.jl:80` calls `measure_labels` inside a `lift`, so every window
  **resize** and parameter change re-measures every label.
- Today there are two `measure_one` methods (`src/measure.jl:14-30`):
  - `measure_one(::AbstractString, …)` — `TextMeasure.prepare` → `layout`,
    render-free. Covers `String` and `LaTeXString` (both `<: AbstractString`).
  - `measure_one(label, …)` — catch-all whose only *working* input is
    `Makie.RichText`. Allocates a throwaway `Scene`, calls `text!`,
    `update_state_before_display!`, then reads `full_boundingbox(t, :pixel)` and
    scales by `ppu`. Correct for RichText, but allocates a `Scene` and runs
    Makie's layout machinery per label, every resize. (For non-string,
    non-RichText input — `Symbol`, numbers — this path already **errors**:
    Makie's `convert_text_string!` has no method for those types. Verified by
    REPL probe.)
- Issue #6 was blocked on `jowch/TextMeasure.jl#1` (TextMeasure lacked
  render-free rich-text measurement). **That blocker is resolved**:
  `TextMeasure.measure_bounds(::MakieBackend, ::Makie.RichText) -> TextBounds`
  is on TextMeasure's pushed `origin/main` (`ext/TextMeasureMakieExt.jl:163`),
  tested in `test/test_richtext.jl` against a real Makie render, agreeing to
  `atol=0.5, rtol=2e-3` at `px_per_unit = 1`.
- `Project.toml [sources]` points at the sibling `../TextMeasure.jl` checkout,
  which is on `main` and already carries the API. **This work needs no
  `[sources]` change** (issue #5 stays independent).

## Decisions

### D1 — String method unchanged; rich method swapped; catch-all becomes a clean error

`measure_bounds` dispatches **only** on `Makie.RichText`, not `AbstractString`
— the two tracks are genuinely different TextMeasure subsystems (a flat layout
engine for strings; a tree-walking bbox measurer for styled RichText). So we do
not literally "collapse to one method." We end with three methods:

```julia
# (1) Plain strings (and LaTeXString) → TextMeasure layout engine. UNCHANGED.
function measure_one(label::AbstractString, font, fontsize::Float64, ppu::Float64)
    f = _resolve_font(font)
    backend = TextMeasure.MakieBackend(; font = f, fontsize = fontsize, px_per_unit = ppu)
    lay = TextMeasure.layout(TextMeasure.prepare(backend, String(label)))
    return Vec2f(lay.size[1], lay.size[2])
end

# (2) Rich text → TextMeasure measure_bounds, render-free. NEW (replaces the Scene render).
function measure_one(label::Makie.RichText, font, fontsize::Float64, ppu::Float64)
    f = _resolve_font(font)
    backend = TextMeasure.MakieBackend(; font = f, fontsize = fontsize, px_per_unit = 1.0)
    tb = TextMeasure.measure_bounds(backend, label)
    return Vec2f(tb.size[1] * ppu, tb.size[2] * ppu)
end

# (3) Anything else → a clear error. Replaces the Scene catch-all (which also errored
#     on these types, deeper in Makie). No new capability; just a legible message.
measure_one(label, font, fontsize::Float64, ppu::Float64) =
    throw(ArgumentError("textrepel! labels must be String, LaTeXString, or " *
                        "Makie.RichText; got $(typeof(label))"))
```

Notes baked into D1:

- **LaTeXString** is an `AbstractString`, so it dispatches to method (1),
  unchanged. This is load-bearing (issue #6 names `LaTeXString`) — it is an
  explicit decision, and a test guards it.
- **Heterogeneous vectors** (`["plain", rich("H", subscript("2"))]`) are
  supported: `measure_labels` is an element-wise comprehension
  (`src/measure.jl:9-10`), so each element dispatches independently regardless
  of the vector's element type (`Vector{Any}` or a `Union`).
- The **`font` argument is the default/fallback font**; `measure_bounds`
  resolves per-run fonts from the RichText tree itself, so the single `font`
  arg does not lose per-segment font information.
- `_resolve_font` is **unchanged** and shared by methods (1) and (2).
- Dispatch is unambiguous: `RichText` is not `<: AbstractString`, so (1) and (2)
  are disjoint; the untyped (3) is strictly less specific than both.

The Scene-based body is deleted. There are **no per-file imports to remove**:
`Scene`, `text!`, `full_boundingbox`, etc. all come from the module-level
`using Makie` (`src/MakieTextRepel.jl`). Deleting the body simply drops the
*usages*; `Scene`/`full_boundingbox` then have no remaining use in `src/` but
need no `import` edits. (`text!`, `widths`, `Point2f`,
`update_state_before_display!` remain used elsewhere — do not touch.)

### D2 — `px_per_unit` absorbed at our boundary (rich path only)

`measure_bounds` hard-requires `px_per_unit == 1.0` and throws otherwise. The
reason is real: its multi-line line-drop is a hardcoded 20px stub (Makie's
`apply_lineheight!`) that does not scale with ppu, so measuring multi-line
RichText at ppu≠1 would skew heights (`ext/TextMeasureMakieExt.jl:165-168`).

We **absorb** this in the rich path: always build the backend at `px_per_unit =
1.0` and apply `ppu` ourselves as a post-scale (`tb.size .* ppu`). The
post-scale *form* is identical to what the old fallback did
(`full_boundingbox(:pixel) * ppu`, where `:pixel` is logical pixels), so the raw
`ArgumentError` is **never surfaced to any caller**, including a direct caller
passing `ppu ≠ 1`.

**Deliberate asymmetry between the two paths:** the *string* path (method 1)
passes `ppu` straight into the backend (`px_per_unit = ppu`) because TextMeasure's
`layout` honors `px_per_unit` correctly, so no post-scale is needed there. Only
the *rich* path needs the manual `* ppu`, because `measure_bounds` forbids
`ppu ≠ 1`. This difference is intentional — do not "unify" method (1) to a
post-scale form.

`px_per_unit` is a *rasterization* concern, not a *layout* concern. The recipe
hardcodes `measure_labels(…, 1.0)` (`recipe.jl:80`) and solves entirely in
logical pixels; the annotation plug-in path never calls `measure_labels` at all
(it consumes Makie-supplied text bboxes — `annotation_algorithm.jl`). A user
exporting at `save(...; px_per_unit = 2)` is rasterized by Makie at save time
and **never drives `measure_bounds` at ppu≠1**. So the plotting path cannot hit
the guard.

### D3 — Acceptance via a golden test against the *old* algorithm

We do not keep a Scene fallback in production. We prove equivalence in the test
layer: replicate the **old algorithm exactly** inside the test and compare the
new `measure_one` result to it. Helper:

```julia
# Test-local. Runs the exact old Scene path; returns (w, h) in LOGICAL pixels.
# Only valid for inputs the old path could render (NOT rich("") — see below).
function old_measure(label, font, fontsize)
    sc = Scene(size = (10, 10))
    t  = text!(sc, Point2f(0, 0); text = label, font = Makie.to_font(font),
               fontsize = Float32(fontsize))
    Makie.update_state_before_display!(sc)
    w = Makie.widths(Makie.full_boundingbox(t, :pixel))
    return (w[1], w[2])
end
```

Note we use `full_boundingbox` (what the old code used), not `boundingbox`. The
borrowed tolerance `atol=0.5, rtol=2e-3` comes from TextMeasure's oracle, which
compares against `boundingbox` — but for a single `text!` plot in `:pixel`
space `boundingbox` delegates to `full_boundingbox`, so they coincide and the
tolerance transplant is valid. We do **not** claim "byte/float-identical": the
new path is glyph-advance arithmetic and the old path is Makie's renderer, so
they match to sub-pixel, not to the bit. The earlier "≤ 1px" phrasing is dropped.

## Behavior changes

- **Empty rich text now works (improvement).** `rich("")` crashes Makie's
  render path on this Makie version ("TypeError in GlyphCollection" —
  documented in `TextMeasure/test/test_richtext.jl`), so the *old* Scene
  fallback already crashed on it. `measure_bounds` handles it (returns a
  degenerate `(0,0)` box). Net: strictly more robust.
- **Unsupported label types (`Symbol`, numbers, …) still error, more cleanly.**
  The old Scene path already errored on these (Makie has no
  `convert_text_string!` for them). The new method (3) raises a legible
  `ArgumentError` at `measure_one` instead of an opaque `MethodError` deep in
  Makie. This is **not** a regression — both error; the message just improves.
  We deliberately do **not** add `Symbol`/`Number` support (out of scope for #6).

## Known limitations (carried, not introduced)

- **Multi-line rich text at `ppu ≠ 1`**: the 20px line-drop is scaled
  externally by `ppu`, which may diverge sub-pixel from a true Makie render at
  that ppu. Matches today's fallback behavior, is unreachable from the recipe
  (always ppu=1), and is an upstream TextMeasure refinement if it ever matters.
- **`\n` inside a `subsup`/`left_subsup` child throws `ArgumentError`** — Makie
  itself errors on this, and `measure_bounds` mirrors it
  (`test_richtext.jl:63-67`). Consistent with a real render; not a regression.
- **Glyph fallback** (`measure.jl:13` TODO): routing through `text_bb` when the
  resolved font lacks a label's glyphs is issue #3. Out of scope.

## Testing

`test/test_measure.jl` — keep the plain-string assertions; upgrade the
rich-text case from "finite & positive" to a **golden comparison** against the
old algorithm via the `old_measure` helper above (invoked only on inputs the
old path could render — i.e. every case except `rich("")`).

Golden cases (concrete literals; `font = "TeX Gyre Heros Makie"`, `fontsize =
24.0` unless noted) — assert `Vec2f(measure_one(label, font, 24.0, 1.0)) ≈
Vec2f(old_measure(label, font, 24.0)...)` within `atol = 0.5, rtol = 2e-3`:

- `rich("Hello, world")` — simple.
- `rich("H", subscript("2"), "O")` — subscript.
- `rich("x", superscript("2"))` — superscript.
- `rich("big ", rich("small"; fontsize = 12.0))` — mixed size.
- `rich("plain ", rich("other"; font = "TeX Gyre Heros Makie Bold"))` — mixed font.
- `rich("line one\nline two")` — multi-line (ppu = 1 only).
- `rich(" ")` — whitespace-only (matches Makie exactly per upstream).
- A **heterogeneous vector** `["Mauna Kea", rich("H", subscript("2"), "O")]` via
  `measure_labels` — assert each element matches its own `old_measure`.

ppu ≠ 1 (the path where new and old could differ):

- **Single-line** rich (e.g. `rich("x", superscript("2"))`) at `ppu = 2.0`:
  assert `measure_one(label, font, 24.0, 2.0) ≈ old_measure(label, font, 24.0)
  .* 2.0` within the same tolerance. (The old `:pixel` bbox is ppu-independent,
  so `old_measure(...) .* 2.0` reproduces the old `full_boundingbox(:pixel) *
  ppu` exactly; single-line bounds have no line-drop term, so the post-scale is
  exact.)
- **Multi-line** at ppu ≠ 1 is explicitly **not** golden-tested; add a comment
  pointing at the Known Limitation (line-drop does not scale).

New-path-only / behavior assertions (old path can't be the oracle here):

- `rich("")` — assert the new path returns a **finite, non-negative** box (old
  path crashes Makie, so no golden compare).
- Unsupported type — `@test_throws ArgumentError measure_one(:Hello, font,
  24.0, 1.0)`. This proves the Scene catch-all is gone (replaced by the throwing
  method 3) and is the concrete proxy for AC-1 ("no `Scene` in `measure_labels`").
- `LaTeXString` — assert it dispatches to the string path (e.g. matches
  `text_bb` of its content), guarding the `LaTeXString <: AbstractString`
  assumption.

Run the suite once per the CLAUDE.md test loop (tee to an agent-scoped log under
`test/output/`, then grep), since `Pkg.test()` pays precompilation each run.

## Acceptance criteria (from #6)

- [ ] Rich-text labels measured render-free — no `Scene` allocation in
  `measure_labels` (the only `Scene` usage in `src/` is deleted; the
  `@test_throws ArgumentError` on an unsupported type is the proxy proving the
  Scene catch-all is gone).
- [ ] New measurements sub-pixel (`atol=0.5, rtol=2e-3`) to today's
  render-based numbers on the representative rich-text set, at ppu=1 and (for
  single-line) ppu=2 (golden test).
- [ ] `textrepel!` rich-text behavior unchanged for users — covered
  *transitively* by measurement equivalence (no separate recipe-level
  regression test is added; the change is confined to the measurement layer).

## Out of scope

- `[sources]` URL flip (#5), glyph fallback (#3), `Symbol`/`Number` label
  support, and `prepare`/`layout` ↔ `measure_bounds` unification (would need an
  upstream `measure_bounds(::MakieBackend, ::AbstractString)` — a legitimate
  future cleanup, deliberately deferred).

## Implementation gotcha

Per project memory: running `Pkg.resolve()` inside a `.claude/worktrees/`
checkout rewrites the TextMeasure `[sources]` path away from the sibling
(caught in PR #11). If implementing in a worktree, use the symlink workaround.
