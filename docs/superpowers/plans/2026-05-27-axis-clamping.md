# Axis Clamping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep repelled labels inside the axis plotting area so they don't clip past the edges, by clamping each label to the axis viewport inside the solver (ggrepel/adjustText-style), on by default.

**Architecture:** A pure `clamp_box_offset` geometry helper; the solver gains an optional `bounds` field on `RepelParams` and clamps each padded label box inside it every iteration, plus step-cap cooling so edge-crowded cases converge instead of limit-cycling; the recipe feeds the scene viewport (size only, origin discarded — `:pixel` anchors are scene-local) into the solve `lift` as `bounds`, so labels re-solve and re-clamp reactively on resize.

**Tech Stack:** Julia 1.11+, Makie 0.24 / CairoMakie 0.15, GeometryBasics, LinearAlgebra.

**Spec:** `docs/superpowers/specs/2026-05-27-axis-clamping-design.md`

---

## Task 1: Pure `clamp_box_offset` helper

**Files:**
- Modify: `src/geometry.jl`
- Modify: `test/test_geometry.jl`

- [ ] **Step 1: Write the failing test**

First, at the TOP of `test/test_geometry.jl`, extend the existing import line so it also
brings in `clamp_box_offset`:

```julia
using MakieTextRepel: box_at, overlap_push, point_push, clip_to_box_edge, clamp_box_offset
```

(`using` cannot go inside a `@testset` block — it must stay at the file's top level.)
Then append this new testset:

```julia
@testset "clamp_box_offset" begin
    bounds = Rect2f(0, 0, 100, 100)

    # fully inside → zero shift
    @test clamp_box_offset(Rect2f(10, 10, 20, 20), bounds) == Vec2f(0, 0)
    # over the right edge → pushed left by the overshoot
    @test clamp_box_offset(Rect2f(90, 10, 20, 20), bounds) ≈ Vec2f(-10, 0)
    # over the bottom edge → pushed up
    @test clamp_box_offset(Rect2f(10, -5, 20, 20), bounds) ≈ Vec2f(0, 5)
    # over left and top → pushed right and down
    @test clamp_box_offset(Rect2f(-5, 90, 20, 20), bounds) ≈ Vec2f(5, -10)
    # wider than bounds on x → pinned to the lower (left) edge: origin.x → 0
    @test clamp_box_offset(Rect2f(20, 10, 200, 20), bounds) ≈ Vec2f(-20, 0)
end
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL — `UndefVarError: clamp_box_offset not defined`.

- [ ] **Step 3: Implement the helper in `src/geometry.jl`**

Append:

```julia
# Per-axis corrective shift to bring one interval inside another, preserving width.
# If the box is wider than the bounds on this axis, pin it to the lower edge.
function _clamp_axis(lo, hi, blo, bhi, w, bw)
    w  > bw  && return blo - lo   # larger than bounds → pin lower edge
    lo < blo && return blo - lo   # over lower edge → push toward +
    hi > bhi && return bhi - hi   # over upper edge → push toward -
    return 0f0
end

"""
Minimal shift to bring `box` fully inside `bounds`, preserving its size. Returns a
zero vector if it already fits. If `box` is larger than `bounds` on an axis, pins it
to that axis's lower edge.
"""
function clamp_box_offset(box::Rect2f, bounds::Rect2f)
    lo, hi   = box.origin, box.origin .+ box.widths
    blo, bhi = bounds.origin, bounds.origin .+ bounds.widths
    sx = _clamp_axis(lo[1], hi[1], blo[1], bhi[1], box.widths[1], bounds.widths[1])
    sy = _clamp_axis(lo[2], hi[2], blo[2], bhi[2], box.widths[2], bounds.widths[2])
    return Vec2f(sx, sy)
end
```

- [ ] **Step 4: Run test to verify it passes**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS (clamp_box_offset testset green; all prior tests still green).

- [ ] **Step 5: Commit**

```bash
git add src/geometry.jl test/test_geometry.jl
git commit -m "Add clamp_box_offset geometry helper"
```

---

## Task 2: `bounds` field + clamp in the solver loop

**Files:**
- Modify: `src/solver.jl`
- Modify: `test/test_solver.jl`

- [ ] **Step 1: Write the failing test**

Append to `test/test_solver.jl`:

```julia
@testset "solve_repel clamping" begin
    bounds = Rect2f(0, 0, 200, 120)
    pad = 4.0
    # padded box of label i at its solved offset
    pbox(anchors, offsets, sizes, i) =
        box_at(anchors[i], offsets[i], sizes[i] .+ Vec2f(2f0 * Float32(pad)))

    # anchors hugging the edges/corners; every final padded box must stay inside
    anchors = [Point2f(5, 5), Point2f(195, 5), Point2f(5, 115),
               Point2f(195, 115), Point2f(100, 60)]
    sizes = fill(Vec2f(40, 16), 5)
    offsets, _ = solve_repel(anchors, sizes, RepelParams(box_padding = pad, bounds = bounds))
    for i in eachindex(anchors)
        b = pbox(anchors, offsets, sizes, i)
        @test b.origin[1] >= -1e-2
        @test b.origin[2] >= -1e-2
        @test b.origin[1] + b.widths[1] <= 200 + 1e-2
        @test b.origin[2] + b.widths[2] <= 120 + 1e-2
    end

    # bounds = nothing is the same as not passing bounds at all (clamp truly off)
    a = solve_repel(anchors, sizes, RepelParams(box_padding = pad, bounds = nothing))[1]
    b = solve_repel(anchors, sizes, RepelParams(box_padding = pad))[1]
    @test a == b

    # degenerate: label wider than bounds → pinned, finite, no NaN
    big, _ = solve_repel([Point2f(100, 60)], [Vec2f(400, 10)],
                         RepelParams(box_padding = 0.0, bounds = bounds))
    @test all(isfinite, big[1])
end
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL — `RepelParams` has no `bounds` field (`MethodError`/`UndefKeywordError`-style failure on the keyword).

- [ ] **Step 3: Add the field and the clamp**

In `src/solver.jl`, add the field to the `RepelParams` struct (place it after `tol`):

```julia
    bounds::Union{Rect2f, Nothing} = nothing   # clamp region in solver (pixel) space; nothing = no clamp
```

Then, in `solve_repel`, replace the **apply-offsets loop** (the block that currently reads
`for i in 1:n; d = _constrain(_clamp_step(Δ[i], smax), p.only_move); offsets[i] = offsets[i] .+ d; maxmove = max(maxmove, norm(d)); end`)
with this version, which folds the clamp into the same update so its shift counts toward `maxmove`:

```julia
        maxmove = 0f0
        for i in 1:n
            d = _constrain(_clamp_step(Δ[i], smax), p.only_move)
            newoff = offsets[i] .+ d
            if p.bounds !== nothing
                box = box_at(anchors[i], newoff, psizes[i])
                newoff = newoff .+ clamp_box_offset(box, p.bounds)
            end
            move = newoff .- offsets[i]
            offsets[i] = newoff
            maxmove = max(maxmove, norm(move))
        end
```

(`psizes` — the padded sizes — and `smax` are already defined earlier in `solve_repel`.)

- [ ] **Step 4: Run test to verify it passes**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS (clamping testset green; all prior solver tests still green — they use `bounds = nothing` so behavior is unchanged).

- [ ] **Step 5: Commit**

```bash
git add src/solver.jl test/test_solver.jl
git commit -m "Clamp label boxes to bounds in the solver"
```

---

## Task 3: Step-cap cooling (convergence under edge-crowding)

**Files:**
- Modify: `src/solver.jl`
- Modify: `test/test_solver.jl`

**Why:** With clamping, tightly edge-crowded labels pinned against a boundary settle into a period-2 limit cycle (overlap-push vs. spring) and spin to `max_iter` instead of converging. Linearly decaying the per-iteration step cap kills the oscillation. This is a deterministic, general improvement (no new parameter) and does not affect the non-crowded cases that already converge in a few iterations.

- [ ] **Step 1: Write the failing test**

Append to `test/test_solver.jl`:

```julia
@testset "solve_repel converges under edge-crowding" begin
    # Wide labels crammed into a small box → they crowd against the walls. Without
    # step-cap cooling this limit-cycles; with it, the solution settles. We detect
    # convergence by stability: one extra iteration changes the result negligibly.
    bounds = Rect2f(0, 0, 80, 48)   # small enough that labels crowd the walls
    anchors = [Point2f(12, 12), Point2f(45, 12), Point2f(78, 12),
               Point2f(28, 43), Point2f(62, 43)]
    sizes = fill(Vec2f(46, 18), 5)
    pa = RepelParams(box_padding = 2.0, bounds = bounds, max_iter = 3000)
    pb = RepelParams(box_padding = 2.0, bounds = bounds, max_iter = 3001)
    oa = solve_repel(anchors, sizes, pa)[1]
    ob = solve_repel(anchors, sizes, pb)[1]
    @test maximum(norm.(oa .- ob)) < 1.0   # converged, not limit-cycling
    @test all(o -> all(isfinite, o), oa)
end
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL — without cooling the crowded case limit-cycles, so `oa` (even iter count) and `ob` (odd) differ by the cycle amplitude. (Validated: `maximum(norm.(oa .- ob)) ≈ 10.0` without cooling for this scenario, vs `≈ 0.098` with cooling.) If it does NOT fail, the chosen scenario doesn't actually limit-cycle on this machine — STOP and report so the scenario can be retuned (do not weaken the assertion to force a pass).

- [ ] **Step 3: Add step-cap cooling**

In `src/solver.jl` `solve_repel`: the per-iteration step cap is currently the constant `smax` (a `Float32` defined before the loop from `p.step_max`). Make it decay with the iteration index.

Change the loop header from `for _ in 1:p.max_iter` to `for it in 1:p.max_iter`, and at the top of the loop body compute the cooled cap, shadowing the constant:

```julia
    for it in 1:p.max_iter
        # Step-cap cooling: linearly decay the per-iteration move cap so crowded,
        # wall-pinned labels settle instead of limit-cycling. Deterministic.
        smax = smax0 * max(0f0, 1f0 - Float32(it) / Float32(p.max_iter))
        # ... existing body (force accumulation, then the apply-offsets loop) ...
```

To make this work, rename the pre-loop constant: change the existing line
`smax = Float32(p.step_max)` to `smax0 = Float32(p.step_max)`. The apply-offsets loop keeps using `smax` (now the cooled per-iteration value) in `_clamp_step(Δ[i], smax)`.

- [ ] **Step 4: Run test to verify it passes**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS (convergence testset green; all prior tests still green — determinism holds because the schedule is deterministic, and the non-crowded cases converge well before cooling bites).

- [ ] **Step 5: Commit**

```bash
git add src/solver.jl test/test_solver.jl
git commit -m "Add step-cap cooling so edge-crowded labels converge"
```

---

## Task 4: Feed the axis viewport into the recipe

**Files:**
- Modify: `src/recipe.jl`
- Modify: `test/test_integration.jl`

- [ ] **Step 1: Write the failing test**

Append to `test/test_integration.jl`:

```julia
@testset "clamping keeps labels inside the axis viewport" begin
    fig = Figure(size = (420, 360)); ax = Axis(fig[1, 1])
    pts = Point2f[(0, 0), (1, 0), (0, 1), (1, 1), (0.5, 0.5)]
    labels = ["alphalabel", "betalabel", "gammalabel", "deltalabel", "epsilonlabel"]
    pl = textrepel!(ax, pts; text = labels)
    Makie.update_state_before_display!(fig.scene)

    offs = pl.computed_offsets[]
    vp = Makie.widths(Makie.viewport(ax.scene)[])          # scene-local size
    sizes = MakieTextRepel.measure_labels(labels, pl.font[], pl.fontsize[], 1.0)
    pad = Float32(pl.box_padding[])
    for i in eachindex(pts)
        apx = Makie.project(ax.scene, :data, :pixel, pts[i])
        anchor = Point2f(apx[1], apx[2])
        box = MakieTextRepel.box_at(anchor, offs[i], sizes[i] .+ Vec2f(2pad))
        @test box.origin[1] >= -1.0
        @test box.origin[2] >= -1.0
        @test box.origin[1] + box.widths[1] <= vp[1] + 1.0
        @test box.origin[2] + box.widths[2] <= vp[2] + 1.0
    end
end
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: FAIL — without clamping wired in, the edge labels' boxes extend past the viewport (right/top assertions fail).

- [ ] **Step 3: Wire the viewport into the solve lift**

In `src/recipe.jl`, inside `Makie.plot!(p::TextRepel)`, immediately AFTER the `register_projected_positions!(...)` call and BEFORE the `solved = lift(...)` block, add the bounds observable:

```julia
    # Clamp region: the axis data-area viewport, SIZE ONLY. The viewport Rect2i carries
    # a figure-relative origin, but :pixel anchors are scene-local (origin at the axis
    # lower-left), so we use (0,0)–widths and discard the origin.
    bounds_obs = lift(Makie.viewport(Makie.parent_scene(p))) do vp
        Rect2f(0, 0, Float32.(widths(vp))...)
    end
```

Then add `bounds_obs` as the final input to the `solved = lift(...)` call, add a matching
final argument `bnds` to its `do` block, and pass `bounds = bnds` into the `RepelParams`
constructor. The full updated block:

```julia
    solved = lift(p.px_anchors, p.text, p.fontsize, p.font,
                  p.force, p.force_point, p.force_pull, p.max_iter, p.only_move,
                  p.box_padding, p.point_padding, p.max_overlaps, bounds_obs) do px, labels, fs, font,
                                                                                 fr, frp, fpl, mi, om,
                                                                                 bp, pp, mo, bnds
        anchors = [Point2f(q[1], q[2]) for q in px]
        sizes = measure_labels(labels, font, fs, 1.0)
        params = RepelParams(; force = Tuple(Float64.(fr)),
                               force_point = Tuple(Float64.(frp)),
                               force_pull = Tuple(Float64.(fpl)),
                               max_iter = Int(mi), only_move = Symbol(om),
                               box_padding = Float64(bp), point_padding = Float64(pp),
                               max_overlaps = Float64(mo), bounds = bnds)
        offsets, dropped = solve_repel(anchors, sizes, params)
        (; anchors, sizes, offsets, dropped)
    end
```

- [ ] **Step 4: Run test to verify it passes**

Run: `julia --project=. -e 'using Pkg; Pkg.test()'`
Expected: PASS (viewport-clamp testset green; all prior tests still green). A benign "Could not find font regular" warning is expected.

- [ ] **Step 5: Commit**

```bash
git add src/recipe.jl test/test_integration.jl
git commit -m "Feed axis viewport into solver so labels clamp to the axis"
```

---

## Task 5: Regenerate the README demo without manual limits

**Files:**
- Modify: `examples/readme_example.jl`
- Modify (regenerate): `assets/example.png`

- [ ] **Step 1: Remove the manual limit padding, rely on clamping**

In `examples/readme_example.jl`, delete the manual frame block:

```julia
# Shared, roomy frame so repelled labels stay in view (and both panels match).
pad = 0.7
xl = (minimum(xs) - pad, maximum(xs) + pad)
yl = (minimum(ys) - pad, maximum(ys) + pad)
```

and the four `xlims!(...)`/`ylims!(...)` calls. Replace the frame handling by giving both
axes a modest autolimit margin (breathing room) and letting clamping keep labels in view.
The body becomes:

```julia
fig = Figure(size = (1000, 480), fontsize = 15)

ax1 = Axis(fig[1, 1]; title = "text! (overlapping)", aspect = 1,
           xautolimitmargin = (0.12, 0.12), yautolimitmargin = (0.12, 0.12))
scatter!(ax1, xs, ys; color = :tomato, markersize = 9)
text!(ax1, xs, ys; text = labels, align = (:left, :bottom), fontsize = 13)

ax2 = Axis(fig[1, 2]; title = "textrepel! (resolved)", aspect = 1,
           xautolimitmargin = (0.12, 0.12), yautolimitmargin = (0.12, 0.12))
scatter!(ax2, xs, ys; color = :tomato, markersize = 9)
textrepel!(ax2, xs, ys; text = labels, fontsize = 13)
```

- [ ] **Step 2: Regenerate the image**

Run: `julia --project=. examples/readme_example.jl`
Expected: prints `wrote .../assets/example.png`, no error (benign font warning ok).

- [ ] **Step 3: Visually confirm**

Open `assets/example.png`. Confirm: in the right panel, labels are spread out AND none are clipped at the axis edges (the whole point — previously edge labels like "node 6"/"node 9" clipped without the manual `xlims!`). If labels still clip, STOP and report (the clamp wiring or viewport frame may be off).

- [ ] **Step 4: Commit**

```bash
git add examples/readme_example.jl assets/example.png
git commit -m "Regenerate README demo: clamping keeps labels in frame without manual limits"
```

---

## Notes / deferred

- **Own-point repulsion** (labels not covering their own markers) — tracked in
  [jowch/MakieTextRepel.jl#1](https://github.com/jowch/MakieTextRepel.jl/issues/1).
- **Overflow opt-out** attribute — deferred until a consumer needs it.
- **Axis expansion** philosophy — explicitly not built (clamp only).
