# Reuse Text Measurements Across Recipe Updates — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop `textrepel!` from re-measuring label text when only positions/solve-params change, by splitting the recipe's fused measure+solve `lift` into two reactive nodes.

**Architecture:** In `Makie.plot!(p::TextRepel)`, extract a `measured_sizes` compute node depending only on the three measure-invalidating inputs (`text`, `fontsize`, `font`); feed it into the existing `solved` node (which keeps all solve-only inputs). Makie's reactive graph then reuses the cached sizes whenever measurement provably can't change. The reuse is asserted via **object identity** on the already-exposed `computed_sizes` — no production test-state.

**Tech Stack:** Julia 1.11, Makie 0.24 (ComputeGraph + `lift`), TextMeasure.jl (sibling checkout), CairoMakie (test-only).

**Spec:** `docs/superpowers/plans/2026-06-18-measure-reuse-spec.md` · **Issue:** #25

## Global Constraints

- **Byte-identity:** for any single static plot, computed offsets/dropped must equal the pre-change output. The "#12 structural defense" test (`test/test_integration.jl:225`) and the coincident-fan-out test (`:241`) must keep passing **unchanged**.
- **No production test-state.** The measurement-reuse property is verified by object identity on the existing `computed_sizes` node — do NOT add counters, flags, or other test-only state to `src/`.
- **No new dependencies.** No change to `Project.toml [deps]` or `[sources]`. **Do not `git add Project.toml`** — the `../TextMeasure.jl` relative source resolves via the `.claude/worktrees/TextMeasure.jl` symlink; a stray Pkg resolve may rewrite the path, and that rewrite must never be committed. Commit only the named files in each task.
- **Keep the layers Makie-free where they already are.** `measure.jl` is not modified by this plan.
- **Warm-start (`init_state`) is OUT of scope** — it would change outputs and break byte-identity. It is tracked by **issue #24** (recipe warm-start). #24 and this plan are complementary and touch mostly different surfaces; the only shared region is the recipe solve node, where this split makes warm-start *easier* to add later. Do not couple the two — keep the solve here a fresh solve.
- All distances are pixels; `ppu` stays hardcoded `1.0` on the recipe path.

## Testing convention (project-mandated)

Per `CLAUDE.md`: precompilation makes each `Pkg.test()` cost minutes. Run the suite once, tee to an agent-scoped log, then grep — do **not** re-run between greps. Baseline is **721/721 passing** (captured `test/output/baseline-measure-reuse.log`).

```bash
cd /home/jonathanchen/projects/MakieTextRepel.jl/.claude/worktrees/measure-reuse
LOG="test/output/test-<your-agent-id>.log"   # test/output/ is gitignored
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -E "Test Summary|Fail|Error" "$LOG"
```

---

### Task 1: Split the recipe's measure/solve lift

**Files:**
- Modify: `src/recipe.jl:75-111` (split one `lift` into two nodes)
- Test: `test/test_integration.jl` (append reuse testset)

**Interfaces:**
- Produces: a `measured_sizes` local node (`Vector{Vec2f}`) feeding `solved`. The `solved` named tuple is unchanged: `(; anchors, sizes, offsets, dropped, params)`. All `computed_*` nodes (`src/recipe.jl:115-119`) and downstream render nodes keep their current values. `computed_sizes` returns the *same object* across position-only updates (measurement reused) and a *new object* across text/fontsize/font changes.

- [ ] **Step 1: Write the failing reuse test**

Append to `test/test_integration.jl`:
```julia
@testset "measurement reuse across position updates (#25)" begin
    fig = Figure(size = (400, 400))
    ax = Axis(fig[1, 1])
    pos = Observable(Point2f[(1, 1), (2, 2), (1.5, 2.5)])
    pl = textrepel!(ax, pos; text = ["alpha", "beta", "gamma"])
    Makie.update_state_before_display!(fig.scene)

    # The measured-sizes vector is threaded unchanged through `solved`. If the split
    # reuses measurements, `computed_sizes` returns the SAME object (===) after a
    # position-only update, and a NEW object after a text change. Object identity is
    # the signal — no production test-state needed.
    sizes_before = pl.computed_sizes[]

    # Mutate ONLY positions → solver re-runs, measurement must be REUSED (same object).
    pos[] = Point2f[(1.2, 1.1), (2.1, 2.2), (1.4, 2.6)]
    Makie.update_state_before_display!(fig.scene)
    @test pl.computed_sizes[] === sizes_before
    @test length(pl.computed_offsets[]) == 3          # solve still ran
    @test all(o -> all(isfinite, o), pl.computed_offsets[])

    # Mutate text → measurement MUST refresh (new object).
    pl.text[] = ["alpha", "beta", "DELTAdelta"]
    Makie.update_state_before_display!(fig.scene)
    @test pl.computed_sizes[] !== sizes_before
    @test length(pl.computed_sizes[]) == 3
end
```

- [ ] **Step 2: Run and verify it fails**

Run the suite (tee to `$LOG`), then:
```bash
grep -nA4 "measurement reuse across position" "$LOG"
```
Expected: the `@test pl.computed_sizes[] === sizes_before` FAILS — today the fused lift re-measures on every update, producing a fresh `Vector` object each time, so identity does not hold.

- [ ] **Step 3: Split the lift**

In `src/recipe.jl`, replace the block at lines 75-111 (the comment `# 2. Measure + solve…` through the closing `end` of the `solved` lift) with:
```julia
    # 2a. Measure label boxes. Depends ONLY on text/fontsize/font — the three
    #     measure-invalidating inputs (#25) — so it is NOT re-run when positions
    #     or any solve-only param change (the movie/animation reuse case). Makie's
    #     reactive graph reuses the cached sizes whenever measurement can't change.
    measured_sizes = lift(p.text, p.fontsize, p.font) do labels, fs, font
        measure_labels(labels, font, fs, 1.0)
    end

    # 2b. Solve. Consumes the cached `measured_sizes`; re-runs on anchor/param
    #     changes while reusing measurements. `sizes` is threaded back into the
    #     result tuple so every downstream node (box_rects, seg_points,
    #     computed_sizes) is unchanged.
    solved = lift(p.px_anchors, measured_sizes,
                  p.force, p.force_point, p.force_pull, p.max_iter, p.only_move,
                  p.box_padding, p.point_padding, p.max_overlaps, bounds_obs,
                  p.min_segment_length, p.markersize) do px, sizes, fr, frp, fpl, mi, om,
                                                          bp, pp, mo, bnds, ml, ms
        anchors = [Point2f(q[1], q[2]) for q in px]
        # markersize (sibling scatter) overrides point_padding when set. textrepel!
        # draws no markers; this only declares the sibling size for clearance.
        eff_pp = if ms === nothing
            Float64(pp)
        elseif ms isa Real
            Float64(ms) / 2 + 0.5
        else
            throw(ArgumentError("textrepel!: `markersize` must be a scalar Real or nothing; got $(typeof(ms)). Per-point marker sizes are not supported — set `point_padding` directly."))
        end
        params = RepelParams(; force = Tuple(Float64.(fr)),
                               force_point = Tuple(Float64.(frp)),
                               force_pull = Tuple(Float64.(fpl)),
                               max_iter = Int(mi), only_move = Symbol(om),
                               box_padding = Float64(bp), point_padding = eff_pp,
                               max_overlaps = Float64(mo), bounds = bnds,
                               min_segment_length = Float64(ml))
        # `bounds_obs` always yields a Rect2f, so `bnds` is never `nothing` here.
        # `bounds = bnds` is set in `params` (exposed as `computed_params`) AND passed
        # positionally; `solve_cluster` overrides params.bounds from the positional arg,
        # so the two agree. Full placement strategy lives in the seam (voronoi-seed →
        # side-select → crossing-repair → constraint-projection legalize).
        offsets, dropped, _, _ = solve_cluster(ProjectionSolver(params), anchors, sizes, bnds)
        (; anchors, sizes, offsets, dropped, params)
    end
```

- [ ] **Step 4: Run and verify the reuse test passes AND byte-identity holds**

Run the suite (tee to `$LOG`), then:
```bash
grep -nA4 "measurement reuse across position" "$LOG"
grep -nA2 "#12 structural defense" "$LOG"
grep -E "Test Summary" "$LOG"
```
Expected: "measurement reuse" passes; the #12 structural-defense and coincident-fan-out testsets still pass; overall summary shows the prior 721 plus the new testset, 0 failures / 0 errors.

- [ ] **Step 5: Commit** (do NOT add Project.toml)

```bash
git add src/recipe.jl test/test_integration.jl
git commit -m "recipe: split measure/solve lift so positions reuse measurements (#25)"
```

---

### Task 2: Animation example + docs note

**Files:**
- Create: `examples/animation_reuse.jl`
- Modify: `README.md` (add a short "Animations" note) and `docs/algorithm.md` (one line on the reuse property)

**Interfaces:**
- Consumes: the public `textrepel!` recipe; mutating `positions[]` reuses measurements (Task 1), observable as `computed_sizes` object identity.

- [ ] **Step 1: Write the example**

Create `examples/animation_reuse.jl`:
```julia
# examples/animation_reuse.jl
# Demonstrates the measure-once / layout-many property (#25): in an animation the label
# text is constant, so positions update every frame and the labels are re-placed —
# WITHOUT re-measuring. The reuse is structural: the recipe keys text measurement on
# `text`/`fontsize`/`font` only (a separate compute node from the solve), so a per-frame
# position change re-solves placement but never re-runs measurement. (The faithful unit
# proof of this lives in the test suite — "measurement reuse across solve-only updates".
# Note: comparing `computed_sizes` object identity does NOT prove reuse here, because
# Makie's ComputeGraph deduplicates equal-valued node outputs.)
using CairoMakie
using MakieTextRepel

fig = Figure(size = (500, 500))
ax = Axis(fig[1, 1], limits = (0, 1, 0, 1), title = "textrepel! animation (measurements reused)")

labels = ["alpha", "beta", "gamma", "delta", "epsilon"]
pos = Observable(Point2f[(0.5, 0.5), (0.52, 0.51), (0.48, 0.49), (0.51, 0.47), (0.49, 0.53)])
scatter!(ax, pos; markersize = 8)
pl = textrepel!(ax, pos; text = labels, markersize = 8)

outfile = joinpath(@__DIR__, "animation_reuse.mp4")
record(fig, outfile, 1:90; framerate = 30) do frame
    t = frame / 90 * 2pi
    # Anchors drift on small circles; text never changes ⇒ no re-measurement.
    pos[] = Point2f[(0.5 + 0.12cos(t + i), 0.5 + 0.12sin(t + i)) for i in 1:length(labels)]
end
@info "wrote $outfile"
```

- [ ] **Step 2: Run the example and confirm it renders without error**

```bash
cd /home/jonathanchen/projects/MakieTextRepel.jl/.claude/worktrees/measure-reuse
julia --project=. examples/animation_reuse.jl 2>&1 | tee test/output/anim-measure-reuse.log
grep -E "wrote .*animation_reuse" test/output/anim-measure-reuse.log && ls -l examples/animation_reuse.mp4
```
Expected: the script completes without error and `animation_reuse.mp4` is written. (If `ffmpeg`/`mp4` is unavailable, switch the output filename to `animation_reuse.gif` — the example is a rendering demo; the faithful reuse proof is the Task 1 unit test, not this script.)

- [ ] **Step 3: Add docs notes**

In `README.md`, add under a new "Animations" subsection:
```markdown
### Animations

`textrepel!` reuses text measurements across reactive updates. In an animation where
the label text is constant, mutating the anchor positions (e.g. `positions[] = …` each
frame) re-solves placement but does **not** re-measure the text — measurement only
re-runs when `text`, `fontsize`, or `font` changes. See `examples/animation_reuse.jl`.
```

In `docs/algorithm.md`, add one line near the recipe/pipeline description:
```markdown
Measurement and solving are separate compute nodes: text measurement is keyed on
`text`/`fontsize`/`font` only, so position-only updates (animations) reuse measurements
(issue #25).
```

- [ ] **Step 4: Verify full suite still green**

```bash
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -E "Test Summary|Fail|Error" "$LOG"
```
Expected: all testsets pass, 0 failures / 0 errors.

- [ ] **Step 5: Commit** (do NOT add Project.toml)

```bash
git add examples/animation_reuse.jl README.md docs/algorithm.md
git commit -m "docs: animation example showing measurement reuse (#25)"
```

---

## Self-Review

- **Spec coverage:** Req 1 (positions don't re-measure) → Task 1 reuse test (`=== sizes_before`). Req 2 (text re-measures) → Task 1 reuse test (`!== sizes_before`). Req 3 (byte-identity) → Task 1 Step 4 keeps #12 + coincident tests green. Req 4 (computed_* / data_limits unchanged) → `solved` tuple and the `computed_*`/`data_limits` blocks are untouched by the split. Req 5 (no new dep; markersize unchanged) → Global Constraints + `eff_pp` logic preserved verbatim.
- **Placeholder scan:** none — every code/command step is concrete.
- **Type consistency:** `measured_sizes :: Vector{Vec2f}` matches `solve_cluster`'s `sizes::Vector{Vec2f}` parameter and the `solved.sizes` field used downstream; `computed_sizes = lift(s -> s.sizes, solved)` (unchanged) returns that object.

## Risks / open question

- **ComputeGraph recompute semantics:** the design assumes that with `measured_sizes` no
  longer listing `px_anchors`/params as inputs, a position-only update does not propagate
  into `measured_sizes`. Validation traced this through Makie 0.24.11 + ComputePipeline and
  confirmed the subgraphs are disjoint; Task 1 Step 2/Step 4 prove it empirically via the
  identity assertion. If Step 4 ever shows `computed_sizes[] !== sizes_before` on the
  position-only mutation, stop and investigate the graph wiring before proceeding (do not
  paper over it by weakening the assertion).
