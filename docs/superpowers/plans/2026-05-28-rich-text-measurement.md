# Render-free Rich-Text Measurement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the `Scene`/`text!`/`full_boundingbox` rich-text fallback in `src/measure.jl` with TextMeasure's render-free `measure_bounds`, proven equivalent to the old numbers by a golden test.

**Architecture:** `measure_one` keeps its `AbstractString` method (TextMeasure layout engine, unchanged), gains a `Makie.RichText` method that calls `TextMeasure.measure_bounds` at `px_per_unit=1` and post-scales by `ppu`, and replaces the catch-all Scene method with a clean `ArgumentError`. Equivalence to the pre-change behavior is locked in by a test that replicates the old Scene algorithm in-test and compares within `atol=0.5, rtol=2e-3`.

**Tech Stack:** Julia 1.11, Makie 0.24, the unregistered TextMeasure.jl (sibling checkout via `Project.toml [sources]`), CairoMakie (test-only backend).

**Spec:** `docs/superpowers/specs/2026-05-28-rich-text-measurement-design.md`

---

## Prerequisites

This work happens in the git worktree at
`.claude/worktrees/issue-6-rich-text-measurement`. Two environment facts matter:

1. **TextMeasure `[sources]` symlink.** `Project.toml` has
   `TextMeasure = {path = "../TextMeasure.jl"}`. From the worktree that relative
   path resolves through an existing symlink
   `.claude/worktrees/TextMeasure.jl -> /home/jonathanchen/projects/TextMeasure.jl`.
   Verify it before running anything:

   ```bash
   ls -la .claude/worktrees/TextMeasure.jl 2>/dev/null || ls -la ../TextMeasure.jl
   ```
   Expected: a symlink pointing at `/home/jonathanchen/projects/TextMeasure.jl`.
   If missing, create it: `ln -s /home/jonathanchen/projects/TextMeasure.jl ../TextMeasure.jl`

2. **Do not let `Pkg.resolve` rewrite `[sources]`.** Per project memory (PR #11),
   resolving inside a worktree can rewrite the relative `[sources]` path. After
   any `Pkg.*` run, check `git diff Project.toml` shows no change to the
   `[sources]` block; if it changed, restore it with
   `git checkout -- Project.toml`.

**Test command used throughout** (run from the worktree root). The suite has no
per-`@testset` filter and `Pkg.test()` pays precompilation, so run it once per
task, tee to an agent-scoped log under the gitignored `test/output/`, then grep:

```bash
mkdir -p test/output
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-issue6.log
grep -nE "measure|Test Summary|Fail|Error|Pass" test/output/test-issue6.log
```

---

## File Structure

- **Modify `src/measure.jl`** — the only production change. Three `measure_one`
  methods (string unchanged, rich swapped, catch-all now throws) plus an updated
  header comment. The single `Scene`/`full_boundingbox` usage in `src/` is here
  and gets deleted.
- **Modify `test/test_measure.jl`** — add new-behavior tests (Task 1) and a
  golden-equivalence suite with a test-local `old_measure` helper (Task 2). Keep
  the existing plain-string assertions untouched.

No other files change. `src/recipe.jl` and `src/annotation_algorithm.jl` are
unaffected (the recipe calls `measure_labels(…, 1.0)`; the annotation path never
calls `measure_labels`).

---

## Task 1: Swap the rich-text measurement path

**Files:**
- Modify: `test/test_measure.jl` (add a new `@testset`; extend the `using` line)
- Modify: `src/measure.jl` (replace the rich method + catch-all; update header)

- [ ] **Step 1: Extend the test imports**

In `test/test_measure.jl`, change the existing import line:

```julia
using MakieTextRepel: measure_labels
```
to:
```julia
using MakieTextRepel: measure_labels, measure_one
```

- [ ] **Step 2: Write the failing new-behavior tests**

Append this `@testset` to `test/test_measure.jl` (inside the file, after the
existing `@testset "measure_labels" begin … end` block):

```julia
@testset "rich-text robustness (new render-free path)" begin
    font = "TeX Gyre Heros Makie"

    # rich("") crashes the old Scene render on this Makie version; the new
    # measure_bounds path returns a degenerate-but-finite box.
    esize = only(measure_labels([rich("")], font, 24.0, 1.0))
    @test all(isfinite, esize)
    @test esize[1] >= 0 && esize[2] >= 0

    # Unsupported label types raise a clear ArgumentError (not an opaque
    # Scene/MethodError). This also proves the Scene catch-all is gone.
    @test_throws ArgumentError measure_one(:Hello, font, 24.0, 1.0)
end
```

- [ ] **Step 3: Run the suite and confirm the new tests fail**

```bash
mkdir -p test/output
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-issue6.log
grep -nE "robustness|Fail|Error|Test Summary" test/output/test-issue6.log
```
Expected: the `rich-text robustness` testset reports failures/errors —
`measure_labels([rich("")], …)` errors inside Makie's render path, and the
`@test_throws ArgumentError` fails because the current catch-all throws a
different exception. All other testsets pass.

- [ ] **Step 4: Rewrite `src/measure.jl`**

Replace the **entire** contents of `src/measure.jl` with:

```julia
# measure.jl — pixel box sizes for labels, render-free via TextMeasure.
# Plain strings (and LaTeXString) go through the layout engine; rich text
# (Makie.RichText) through measure_bounds. No Scene/render is allocated.

"""Resolve a Makie font attribute to something TextMeasure/`text_bb` accept."""
_resolve_font(f) = Makie.to_font(f)
_resolve_font(f::Symbol) = Makie.to_font(String(f))

"""Measure every label, returning a `Vector{Vec2f}` of (width, height) in pixels."""
measure_labels(labels, font, fontsize::Real, px_per_unit::Real) =
    [measure_one(lbl, font, Float64(fontsize), Float64(px_per_unit)) for lbl in labels]

# Plain strings (and LaTeXString) → TextMeasure layout engine, render-free.
# The string path passes ppu straight into the backend (layout honors it).
# TODO(glyph-fallback): route through text_bb when the resolved font lacks the label's glyphs.
function measure_one(label::AbstractString, font, fontsize::Float64, ppu::Float64)
    f = _resolve_font(font)
    backend = TextMeasure.MakieBackend(; font = f, fontsize = fontsize, px_per_unit = ppu)
    lay = TextMeasure.layout(TextMeasure.prepare(backend, String(label)))
    return Vec2f(lay.size[1], lay.size[2])
end

# Rich text → TextMeasure measure_bounds, render-free. measure_bounds requires
# px_per_unit == 1 (its line-drop stub does not scale), so we measure at 1 and
# apply ppu as a post-scale — matching the old full_boundingbox(:pixel) * ppu.
function measure_one(label::Makie.RichText, font, fontsize::Float64, ppu::Float64)
    f = _resolve_font(font)
    backend = TextMeasure.MakieBackend(; font = f, fontsize = fontsize, px_per_unit = 1.0)
    tb = TextMeasure.measure_bounds(backend, label)
    return Vec2f(tb.size[1] * ppu, tb.size[2] * ppu)
end

# Any other label type → a clear error. The old Scene path also failed on these
# (Makie has no convert_text_string! for Symbol/Number); this just makes it legible.
measure_one(label, font, fontsize::Float64, ppu::Float64) =
    throw(ArgumentError("textrepel! labels must be String, LaTeXString, or " *
                        "Makie.RichText; got $(typeof(label))"))
```

- [ ] **Step 5: Run the suite and confirm everything passes**

```bash
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-issue6.log
grep -nE "robustness|measure_labels|Fail|Error|Test Summary" test/output/test-issue6.log
git diff --stat Project.toml   # expect: no change to [sources]
```
Expected: `rich-text robustness` passes; the existing `measure_labels` testset
(plain strings + the `rich("H", subscript("2"), "O")` finite/positive check)
still passes; whole suite green. `Project.toml` unchanged.

- [ ] **Step 6: Commit**

```bash
git add src/measure.jl test/test_measure.jl
git commit -m "feat: render-free rich-text measurement via measure_bounds (#6)

Replace the throwaway-Scene fallback in measure.jl with TextMeasure's
measure_bounds for Makie.RichText (px_per_unit=1 + ppu post-scale). The
catch-all now raises a clear ArgumentError instead of failing inside Makie.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: Golden equivalence + edge-case regression suite

This task adds characterization tests that pin the new implementation's output to
the **old** algorithm (replicated in-test) across a representative rich-text set,
plus the ppu≠1, heterogeneous-vector, and LaTeXString-routing cases from the spec.

**Files:**
- Modify: `test/test_measure.jl` (add one `@testset` with a local `old_measure` helper)

- [ ] **Step 1: Add the golden-equivalence testset**

Append this `@testset` to `test/test_measure.jl`:

```julia
@testset "rich-text golden vs old Scene render" begin
    font = "TeX Gyre Heros Makie"

    # Replicates the pre-#6 Scene algorithm exactly; returns (w, h) in LOGICAL
    # pixels. Valid only for inputs the old path could render (NOT rich("")).
    function old_measure(label, fnt, fontsize)
        sc = Scene(size = (10, 10))
        t  = text!(sc, Point2f(0, 0); text = label, font = Makie.to_font(fnt),
                   fontsize = Float32(fontsize))
        Makie.update_state_before_display!(sc)
        w = Makie.widths(Makie.full_boundingbox(t, :pixel))
        return Vec2f(w[1], w[2])
    end

    # Representative rich-text constructs, measured at ppu = 1.
    cases = [
        rich("Hello, world"),                                    # simple
        rich("H", subscript("2"), "O"),                          # subscript
        rich("x", superscript("2")),                             # superscript
        rich("big ", rich("small"; fontsize = 12.0)),            # mixed size
        rich("plain ", rich("other"; font = "TeX Gyre Heros Makie Bold")),  # mixed font
        rich("line one\nline two"),                              # multi-line (ppu=1 only)
        rich(" "),                                               # whitespace-only
    ]
    for lbl in cases
        got  = only(measure_labels([lbl], font, 24.0, 1.0))
        want = old_measure(lbl, font, 24.0)
        @test got[1] ≈ want[1] atol = 0.5 rtol = 2e-3
        @test got[2] ≈ want[2] atol = 0.5 rtol = 2e-3
    end

    # Heterogeneous vector: each element dispatches independently.
    hetero = ["Mauna Kea", rich("H", subscript("2"), "O")]
    sizes  = measure_labels(hetero, font, 24.0, 1.0)
    for (got, lbl) in zip(sizes, hetero)
        want = old_measure(lbl, font, 24.0)
        @test got[1] ≈ want[1] atol = 0.5 rtol = 2e-3
        @test got[2] ≈ want[2] atol = 0.5 rtol = 2e-3
    end

    # ppu != 1: single-line rich post-scales exactly. The old :pixel bbox is
    # ppu-independent, so old_measure(...) .* 2 reproduces the old * ppu, and a
    # single line has no line-drop term, so the post-scale is exact.
    sl    = rich("x", superscript("2"))
    got2  = only(measure_labels([sl], font, 24.0, 2.0))
    want2 = old_measure(sl, font, 24.0) .* 2.0
    @test got2[1] ≈ want2[1] atol = 0.5 rtol = 2e-3
    @test got2[2] ≈ want2[2] atol = 0.5 rtol = 2e-3

    # LaTeXString is an AbstractString → routes to the string path (method 1),
    # not the throwing catch-all. (Behavior is unchanged from before #6.)
    lstr  = Makie.LaTeXStrings.LaTeXString("x^2")
    lsize = only(measure_labels([lstr], font, 24.0, 1.0))
    @test all(isfinite, lsize)
    @test lsize[1] > 0 && lsize[2] > 0
end
```

- [ ] **Step 2: Run the suite and confirm the golden suite passes**

```bash
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-issue6.log
grep -nE "golden|robustness|measure_labels|Fail|Error|Test Summary" test/output/test-issue6.log
git diff --stat Project.toml   # expect: no change to [sources]
```
Expected: `rich-text golden vs old Scene render` passes (every case within
tolerance), all prior testsets still pass, whole suite green. If a specific case
exceeds tolerance, that is a real measurement discrepancy — investigate the
construct against `TextMeasure/test/test_richtext.jl` rather than loosening the
tolerance.

- [ ] **Step 3: Commit**

```bash
git add test/test_measure.jl
git commit -m "test: golden equivalence for render-free rich-text measurement (#6)

Pin measure_one's RichText output to the old Scene algorithm within
atol=0.5,rtol=2e-3 across simple/subscript/superscript/mixed/multi-line/
whitespace cases, plus ppu=2 single-line, heterogeneous vectors, and
LaTeXString routing.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Final Verification

- [ ] **Full suite green.** Confirm the last `Test Summary` in
  `test/output/test-issue6.log` shows 0 failures / 0 errors across all eight
  included test files.
- [ ] **No production Scene measurement.** `grep -n "Scene\|full_boundingbox" src/`
  returns nothing (the only matches were in `measure.jl` and are deleted; the
  recipe's `text!`/`boundingbox` overrides are unrelated and remain).
- [ ] **`[sources]` intact.** `git diff Project.toml` is empty.
- [ ] **Acceptance criteria (issue #6):**
  - Rich-text measured render-free — no `Scene` in `measure_labels` (proxy: the
    `@test_throws ArgumentError` + the `grep` above).
  - New measurements sub-pixel to old numbers (golden suite, ppu=1 and ppu=2).
  - `textrepel!` rich-text behavior unchanged (transitive — measurement layer
    only; no recipe code changed).

---

## Self-Review Notes

- **Spec coverage:** D1 (three methods) → Task 1 Step 4. D2 (ppu post-scale +
  asymmetry) → encoded in the method bodies + the ppu=2 golden case. D3 (golden
  test, pinned tolerance, `full_boundingbox` oracle) → Task 2. Behavior changes
  (empty-rich robustness, ArgumentError) → Task 1 Steps 2/4. Edge cases
  (heterogeneous, LaTeXString, whitespace, multi-line ppu=1) → Task 2.
- **No placeholders:** every code/run step is concrete and paste-ready.
- **Type consistency:** `measure_one`/`measure_labels` signatures
  `(label, font, fontsize::Float64, ppu::Float64)` are identical across the
  production code and every test call; `Vec2f` return type is consistent;
  `old_measure` returns `Vec2f` to match.
- **Out of scope (unchanged):** `[sources]` flip (#5), glyph fallback (#3),
  Symbol/Number support, layout/measure_bounds unification.
