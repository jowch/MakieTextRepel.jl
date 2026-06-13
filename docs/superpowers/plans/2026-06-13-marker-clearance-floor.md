# Marker-Clearance Floor (Point-Aware Legalize) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `legalize` treat every scatter anchor as a fixed keep-out so labels clear markers post-legalize, resolving marker occlusion (`point_overlaps = 1 → 0` on the hero figure) without ever silently dropping a label to satisfy a marker.

**Architecture:** Add a `soft` node class to the pure `legalize` (soft nodes push but are excluded from the drop-triggering `residual`). In `ProjectionSolver.legalize_and_drop`, append every anchor as a fixed+soft keep-out node with signed half-extent `mc = point_padding − box_padding`, yielding exactly `point_padding` text-edge clearance. Bump the marker-clearance default on the two user surfaces only (recipe attr + `TextRepelAlgorithm` ctor), leaving the `RepelParams` primitive (the ForceSolver halo) at `0.0`. Add a recipe-only `markersize` convenience attribute.

**Tech Stack:** Julia 1.11, Makie 0.24, GeometryBasics, CairoMakie (test-only), Test.jl. Pure layers stay Makie-free.

**Spec:** `docs/superpowers/specs/2026-06-13-marker-clearance-floor-design.md` (read it first — §1a soft nodes, §3 default decoupling, §5 floor scope are load-bearing).

**Conventions (from CLAUDE.md):**
- Test runs are expensive (precompile). Run the full suite **once**, tee to an agent-scoped log, grep it; re-run only after a code change. `LOG="test/output/test-<agent-id>.log"` (gitignored).
  `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"`
- For fast unit iteration without `Pkg.test()`, exercise pure layers directly:
  `julia --project=. -i -e 'using MakieTextRepel'` then call `MakieTextRepel.legalize(...)`, `MakieTextRepel.solve_cluster(...)`, etc.
- Commit messages end with the `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>` trailer.
- Distances are pixels. `box_at(anchor, offset, size)` centers a box at `anchor+offset`. `point_covered(p, box, pad)` = strict two-axis AND (`geometry.jl`).

---

## File structure

- `src/legalize.jl` — **modify**: add `soft::Union{Nothing,BitVector}` kwarg to `legalize`; exclude soft pairs from the returned `residual`. (`dykstra!` unchanged.)
- `src/solvers/projection.jl` — **modify**: `legalize_and_drop` appends `n` fixed+soft anchor keep-out nodes; passes `soft` to `legalize`.
- `src/recipe.jl` — **modify**: new `markersize` attribute; bump `point_padding` attr `2.0→5.0`; derive effective `point_padding` in the solve lift; add `markersize` to lift deps; docstrings.
- `src/annotation_algorithm.jl` — **modify**: `point_padding = 5.0` default in the kwarg ctor; docstring.
- `src/params.jl` — **modify**: docstring reframe only (default stays `0.0`).
- `CLAUDE.md` — **modify**: architecture + state notes.
- `examples/readme_example.jl` — **modify**: shared deterministic hero dataset + `markersize = 9`; regenerate `assets/example.png`.
- `test/test_legalize.jl` — **modify**: soft-node + negative-half-extent + pp=0 unit tests.
- `test/test_projection_solver.jl` — **modify**: floor regression, warm-start, pinned, axis-lock, anti-cascade, dropped-anchor tests.
- `test/test_integration.jl` — **modify**: hero `point_overlaps==0` test; fix stale "connectors suppressed" fixture.
- `test/test_annotation_algorithm.jl` — **modify**: ctor-default sweep + comment fix.

---

## Task 1: `legalize` soft-node support

**Files:**
- Modify: `src/legalize.jl` (signature + final residual loop)
- Test: `test/test_legalize.jl`

- [ ] **Step 1: Write the failing tests** — append a new top-level `@testset` block at the end of `test/test_legalize.jl` (the file has no single enclosing testset). These drive `legalize` directly.

```julia
@testset "soft keep-out nodes (#21)" begin
    using MakieTextRepel: legalize
    bounds = Rect2f(0, 0, 200, 200)

    # (a) A fixed node with NEGATIVE half-extent still separates a movable box.
    # node 1: movable label, size 40x20 (psize), centered at its anchor (offset 0).
    # node 2: fixed keep-out at (60,100) with half-extent mc = -? No: use mc>0 here to
    # prove separation; negative-mc behaviour is covered by the gap math in (c).
    anchors = [Point2f(40, 100), Point2f(60, 100)]
    offsets = [Vec2f(0, 0), Vec2f(0, 0)]
    psizes  = [Vec2f(40, 20), Vec2f(2, 2)]      # label vs a tiny (hw=1) fixed node
    fixed   = BitVector([false, true])
    r = legalize(anchors, offsets, psizes, bounds; fixed = fixed)
    # label half-width 20 + node half-width 1 = 21; anchors are 20 apart → must push
    # the label so |Δx| ≥ 21.
    c1 = anchors[1][1] + r.offsets[1][1]
    @test abs(c1 - 60) ≥ 21 - 0.05

    # (b) A SOFT fixed node's residual is excluded, with a non-soft negative control.
    # Lock movement to x (only_move=:x) AND clamp the label against the right bound, so
    # the separation is genuinely unsatisfiable in-bounds (the label cannot escape on y).
    # Without the lock the label would just slide down to clear on y and residual→0,
    # making the negative control vacuous — that bug was caught in plan review.
    bsm     = Rect2f(0, 0, 100, 200)
    anc     = [Point2f(95, 100), Point2f(70, 100)]
    off     = [Vec2f(0, 0), Vec2f(0, 0)]
    psz     = [Vec2f(40, 20), Vec2f(40, 20)]    # label (hw 20) + fixed node (hw 20)
    fx      = BitVector([false, true])
    # only_move=:x ⇒ label clamps to center x = 80 (right edge); node at 70 needs a
    # 40px center gap ⇒ achievable |Δx| = 10 ≪ 40 ⇒ penetration ~20px remains.
    rsoft   = legalize(anc, off, psz, bsm; fixed = fx, soft = BitVector([false, true]),
                       only_move = :x)
    rhard   = legalize(anc, off, psz, bsm; fixed = fx, only_move = :x)  # soft = none
    @test rsoft.offsets[1] != Vec2f(0, 0)        # push still fired (label moved off node)
    @test rsoft.residual ≤ 0.5f0                 # soft pair excluded from residual
    @test rhard.residual > 0.5f0                 # negative control: counted when not soft

    # (c) point_padding=0 ⇒ mc = -box_padding ⇒ gap = unpadded_half: a clean slot is
    # bit-identical (no spurious push). box_padding=4, point_padding=0 ⇒ mc=-4.
    # label psize = unpadded(40,20)+2*4 = (48,28); keep-out node hw = mc = -4.
    # Place the label so its UNPADDED edge sits exactly on the anchor (clean slot):
    # unpadded half-width 20, anchor at 40, label center at 40+20 = 60 ⇒ offset (20,0).
    anc2 = [Point2f(40, 100), Point2f(40, 100)]   # node 2 is the keep-out at the anchor
    off2 = [Vec2f(20, 0), Vec2f(0, 0)]
    psz2 = [Vec2f(48, 28), Vec2f(-8, -8)]         # mc=-4 ⇒ width 2*mc = -8
    fx2  = BitVector([false, true])
    r2   = legalize(anc2, off2, psz2, bounds; fixed = fx2, soft = BitVector([false, true]))
    @test r2.offsets[1] == Vec2f(20, 0)           # bit-identical: no push
end
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-<agent-id>.log` then `grep -nA3 "soft keep-out" test/output/test-<agent-id>.log`
Expected: FAIL — `legalize` has no `soft` keyword (`MethodError`/`got unsupported keyword argument "soft"`).

- [ ] **Step 3: Add the `soft` kwarg and exclude soft pairs from the residual**

In `src/legalize.jl`, change the `legalize` signature:

```julia
function legalize(anchors::Vector{Point2f}, offsets::Vector{Vec2f},
                  psizes::Vector{Vec2f}, bounds::Rect2f;
                  fixed::BitVector, soft::Union{Nothing,BitVector} = nothing,
                  only_move::Symbol = :both, rounds::Int = 400)
```

Immediately after `movable = [!fixed[i] for i in 1:n]` add:

```julia
    softmask = soft === nothing ? falses(n) : soft
    length(softmask) == n || throw(DimensionMismatch(
        "soft length $(length(softmask)) does not match anchors length $n"))
```

In the **final residual loop only** (the `for i in 1:n, j in (i+1):n` block that computes `finalpen`), add the soft skip right after the two-fixed skip:

```julia
    finalpen = 0.0
    for i in 1:n, j in (i+1):n
        (!movable[i] && !movable[j]) && continue
        (softmask[i] || softmask[j]) && continue      # soft keep-out: pushes, but not counted
        ox = (hw[i] + hw[j]) - abs(x[i] - x[j])
        oy = (hh[i] + hh[j]) - abs(y[i] - y[j])
        (ox > 0.01 && oy > 0.01) && (finalpen = max(finalpen, min(ox, oy)))
    end
```

Do **not** touch the constraint-generation loop (lines ~91–106) or `dykstra!` — soft nodes must still generate constraints and push.

Update the `legalize` docstring: add a line documenting `soft` — "`soft[i]` marks a keep-out node that participates in projection (it still pushes movable nodes) but is excluded from the returned `residual`; used for marker keep-outs whose clearance shortfall must not trigger label dropping."

- [ ] **Step 4: Run the tests to verify they pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-<agent-id>.log` then `grep -nA3 "soft keep-out\|Test Summary" test/output/test-<agent-id>.log`
Expected: PASS for the "soft keep-out nodes (#21)" testset; whole suite still green (no other consumer passes `soft`).

- [ ] **Step 5: Commit**

```bash
git add src/legalize.jl test/test_legalize.jl
git commit -m "feat(legalize): add soft keep-out nodes excluded from residual (#21)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 2: ProjectionSolver appends anchor keep-out nodes

**Files:**
- Modify: `src/solvers/projection.jl` (`legalize_and_drop`)
- Test: `test/test_projection_solver.jl`

- [ ] **Step 1: Write the failing floor + behaviour tests** — append to `test/test_projection_solver.jl`. Use the warm-start path (`init_state`) for deterministic, controllable legalize-erasure.

```julia
@testset "marker-clearance floor (#21)" begin
    using MakieTextRepel: ProjectionSolver, RepelParams, solve_cluster, point_covered, box_at
    bounds = Rect2f(0, 0, 200, 200)
    EPS = 0.05

    # --- Floor regression (warm-start legalize-erasure) ---
    # Label 1 warm-started so its box covers foreign anchor 2; label 2 parked away.
    # Pre-keep-out: warm path only legalizes label-vs-label, so anchor 2 stays covered.
    anchors = [Point2f(50, 100), Point2f(80, 100)]
    sizes   = [Vec2f(40, 20),   Vec2f(40, 20)]
    pp = 5.0
    s = ProjectionSolver(RepelParams(; box_padding = 4.0, point_padding = pp))
    # init_state: label 1 centered at (65,100) (covers anchor 2 at x=80,
    # unpadded span x∈[45,85]); label 2 pushed far below.
    init = [Vec2f(15, 0), Vec2f(0, -80)]
    res = solve_cluster(s, anchors, sizes, bounds; init_state = init)
    for i in eachindex(anchors)
        res.dropped[i] && continue
        bi = box_at(anchors[i], res.offsets[i], sizes[i])     # UNPADDED text box
        for j in eachindex(anchors)
            @test !point_covered(anchors[j], bi, pp - EPS)    # own + foreign cleared
        end
    end

    # --- Pinned label on its own marker is bit-identical (exempt) ---
    pin = BitVector([true, false])
    pinoff = [Vec2f(0, 0), Vec2f(0, -80)]     # label 1 sits ON its own anchor
    s2 = ProjectionSolver(RepelParams(; box_padding = 4.0, point_padding = pp))
    res2 = solve_cluster(s2, anchors, sizes, bounds; init_state = pinoff,
                         pin_mask = pin, pinned_offsets = pinoff)
    @test res2.offsets[1] == Vec2f(0, 0)      # pinned: not pushed off its marker
    @test res2.dropped[1] == false            # pinned: not dropped

    # --- only_move=:x, foreign marker covering on both axes → cleared on x, no drop ---
    # BOTH anchors must sit on the SAME side of the label so the single locked x-axis can
    # clear them together. (A label wedged BETWEEN its own anchor and a foreign anchor on
    # the locked axis cannot clear both — that is the documented §5 best-effort
    # degradation, and a fixture asserting clearance there is RED forever. Caught in
    # plan review.) Own anchor (40) and foreign anchor (50) are both LEFT of label 1's
    # settle point; label 1 shoots right to center 75 (span x∈[55,95]), clearing both.
    # Label 2 is parked FAR LEFT IN X (Vec2f(-80,0)); only_move=:x preserves that x, so it
    # never collapses onto / overlaps label 1. This matters: with no label–label push, the
    # ONLY thing that can move label 1 is the marker keep-out — so the test genuinely
    # exercises the floor (verified RED against the pre-keep-out solver). Parking label 2
    # in Y instead (Vec2f(0,-80)) would be zeroed by _constrain(:x) onto label 1 and the
    # label–label push would clear the markers coincidentally, making the test vacuous —
    # empirically confirmed in plan-review verification.
    anchors_x = [Point2f(40, 100), Point2f(50, 100)]
    s3 = ProjectionSolver(RepelParams(; box_padding = 4.0, point_padding = pp,
                                        only_move = :x))
    res3 = solve_cluster(s3, anchors_x, sizes, bounds; init_state = [Vec2f(15, 0), Vec2f(-80, 0)])
    @test count(res3.dropped) == 0
    b1 = box_at(anchors_x[1], res3.offsets[1], sizes[1])
    @test !point_covered(anchors_x[2], b1, pp - EPS)   # foreign cleared on the locked x axis
    @test !point_covered(anchors_x[1], b1, pp - EPS)   # own marker cleared too
end
```

- [ ] **Step 2: Run to verify the floor test fails (red), confirming the bug exists pre-fix**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-<agent-id>.log` then `grep -nA4 "marker-clearance floor" test/output/test-<agent-id>.log`
Expected: FAIL on the floor regression block (`point_covered` true for the foreign anchor) — this is the documented legalize-erasure. If it does **not** fail, the fixture is wrong (the warm box must actually cover anchor 2 within `pp`); adjust `init` until red before proceeding.

- [ ] **Step 3: Append keep-out nodes in `legalize_and_drop`**

In `src/solvers/projection.jl`, inside `solve_cluster`, just **before** the `function legalize_and_drop(...)` definition add the signed half-extent:

```julia
    # Marker keep-out half-extent (signed). A label node carries padded half-extent
    # (unpadded_half + box_padding); a fixed keep-out node of half-extent mc separates
    # the pair to unpadded_half + point_padding ⇒ text edge clears the marker by exactly
    # point_padding (independent of box_padding), matching point_covered.
    mc = Float32(p.point_padding - p.box_padding)
```

Replace the body of `legalize_and_drop`'s `while true` working-array assembly so it allocates `m + k + n` nodes and appends the keep-out nodes last (full anchor list, ascending index, every round):

```julia
        while true
            act = Int[i for i in 1:n if !drp[i]]
            m = length(act); k = length(obstacles)
            tot = m + k + n
            w_anchors = Vector{Point2f}(undef, tot)
            w_offsets = Vector{Vec2f}(undef, tot)
            w_psizes  = Vector{Vec2f}(undef, tot)
            w_fixed   = falses(tot)
            w_soft    = falses(tot)
            for (t, i) in enumerate(act)
                w_anchors[t] = anchors[i]; w_offsets[t] = offs[i]; w_psizes[t] = psizes[i]
                (pin_mask !== nothing && pin_mask[i]) && (w_fixed[t] = true)
            end
            for (t, ob) in enumerate(obstacles)
                w_anchors[m + t] = Point2f(ob.origin .+ ob.widths ./ 2)
                w_offsets[m + t] = Vec2f(0, 0)
                w_psizes[m + t]  = Vec2f(ob.widths)
                w_fixed[m + t]   = true
            end
            # Marker keep-out: every anchor (own + foreign, incl. dropped/pinned), fixed
            # + soft, ascending index → deterministic, round-invariant Dykstra order.
            for i in 1:n
                w_anchors[m + k + i] = anchors[i]
                w_offsets[m + k + i] = Vec2f(0, 0)
                w_psizes[m + k + i]  = Vec2f(2mc, 2mc)
                w_fixed[m + k + i]   = true
                w_soft[m + k + i]    = true
            end
            lz = legalize(w_anchors, w_offsets, w_psizes, bounds;
                          fixed = w_fixed, soft = w_soft, only_move = p.only_move)
            for (t, i) in enumerate(act)
                offs[i] = lz.offsets[t]
            end
            (lz.residual ≤ 0.5f0 || count(!, drp) ≤ 1) && break
            idx = drop_most_overlapped!(drp, anchors, offs, psizes, pin_mask, obstacles)
            idx == 0 && break
        end
```

(`drop_most_overlapped!` is unchanged: marker clearance is best-effort and never drives a drop — the soft nodes are excluded from `lz.residual`, so the drop gate sees only label/obstacle penetration.)

- [ ] **Step 4: Run to verify the floor tests pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-<agent-id>.log` then `grep -nA4 "marker-clearance floor\|Test Summary\|Fail\|Error" test/output/test-<agent-id>.log`
Expected: PASS for "marker-clearance floor (#21)". Other testsets may now shift (later tasks fix fallout); note any failures for Task 7 but do not fix them here unless they are in this testset.

- [ ] **Step 5: Commit**

```bash
git add src/solvers/projection.jl test/test_projection_solver.jl
git commit -m "feat(projection): point-aware legalize — anchor keep-out nodes (#21)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 3: Anti-cascade + dropped-anchor + warm-start own-marker tests

**Files:**
- Test: `test/test_projection_solver.jl`

These lock §1a (marker residual never drops) and the full-anchor-list keep-out. They should pass against the Task 2 implementation (verifying, not red-green driving new code).

- [ ] **Step 1: Write the tests** — append to `test/test_projection_solver.jl`.

```julia
@testset "marker keep-out: anti-cascade + coverage (#21)" begin
    using MakieTextRepel: ProjectionSolver, RepelParams, solve_cluster, point_covered, box_at
    bounds = Rect2f(0, 0, 200, 200)

    # --- Warm-start: a label on its own anchor is pushed off (own-marker floor) ---
    anchors = [Point2f(100, 100)]
    sizes   = [Vec2f(40, 20)]
    pp = 5.0
    s = ProjectionSolver(RepelParams(; box_padding = 4.0, point_padding = pp))
    res = solve_cluster(s, anchors, sizes, bounds; init_state = [Vec2f(0, 0)])
    b = box_at(anchors[1], res.offsets[1], sizes[1])
    @test !point_covered(anchors[1], b, pp - 0.05)     # pushed clear of own marker
    @test res.dropped[1] == false

    # --- Anti-cascade: marker-clearance that is in-bounds-UNSATISFIABLE must NOT drop a
    # label (soft nodes are excluded from the drop-triggering residual, §1a). This is the
    # discriminating test: if a regression let soft nodes count toward residual, the drop
    # loop would fire and shed a label here. Lock to y in a SHORT viewport so neither
    # label can move far enough to clear its own marker, while the two labels are far
    # apart in x (no label-label overlap ⇒ the honest baseline drop count is 0).
    short  = Rect2f(0, 0, 200, 40)               # height 40; label hh=10 ⇒ center y∈[10,30]
    anchors2 = [Point2f(50, 20), Point2f(150, 20)]
    sizes2   = [Vec2f(40, 20), Vec2f(40, 20)]
    # point_padding=15 ⇒ own-marker needs |Δy| ≥ hh+pp = 25, but max achievable is 10 ⇒
    # infeasible on the locked y axis for both labels.
    s2 = ProjectionSolver(RepelParams(; box_padding = 4.0, point_padding = 15.0,
                                        only_move = :y))
    res2 = solve_cluster(s2, anchors2, sizes2, short)
    @test count(res2.dropped) == 0      # soft residual excluded ⇒ no cascade drop
    # And the two labels still do not overlap each other (far apart in x):
    bb1 = box_at(anchors2[1], res2.offsets[1], sizes2[1] .+ 8)
    bb2 = box_at(anchors2[2], res2.offsets[2], sizes2[2] .+ 8)
    @test MakieTextRepel.overlap_push(bb1, bb2) == Vec2f(0, 0)

    # --- Dropped-anchor keep-out participates: a surviving label clears a dropped
    # label's marker. Force a drop via a genuine label-label over-capacity cluster. ---
    # Three labels crammed so one must drop; assert survivors clear ALL anchors' markers.
    anchors3 = [Point2f(100, 100), Point2f(105, 100), Point2f(110, 100)]
    sizes3   = [Vec2f(60, 30), Vec2f(60, 30), Vec2f(60, 30)]
    small    = Rect2f(0, 0, 80, 80)     # too small for 3 big labels → at least one drops
    s3 = ProjectionSolver(RepelParams(; box_padding = 4.0, point_padding = 5.0))
    res3 = solve_cluster(s3, anchors3, sizes3, small)
    for i in eachindex(anchors3)
        res3.dropped[i] && continue
        bi = box_at(anchors3[i], res3.offsets[i], sizes3[i])
        for j in eachindex(anchors3)           # includes dropped j's anchor
            # best-effort in this tight scene; assert no FOREIGN anchor is covered
            j == i && continue
            @test !point_covered(anchors3[j], bi, 5.0 - 0.05) || res3.residual > 0.5f0
        end
    end
end
```

Note: the dropped-anchor block allows a best-effort escape (`|| residual > 0.5`) because the 80×80 scene is deliberately over-capacity; the assertion's purpose is that keep-out nodes for dropped anchors are *present and active*, not that a pathological scene is perfectly cleared.

- [ ] **Step 2: Run to verify they pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-<agent-id>.log` then `grep -nA4 "anti-cascade" test/output/test-<agent-id>.log`
Expected: PASS. If the anti-cascade test drops a label, §1a is violated — re-check that `w_soft` is set on the keep-out nodes and that the residual loop skip from Task 1 is in place.

- [ ] **Step 3: Commit**

```bash
git add test/test_projection_solver.jl
git commit -m "test(projection): anti-cascade + dropped-anchor keep-out coverage (#21)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 4: `markersize` recipe attribute + default bump

**Files:**
- Modify: `src/recipe.jl`
- Test: `test/test_integration.jl`

- [ ] **Step 1: Write the failing tests** — append to `test/test_integration.jl`. These render via CairoMakie and read `computed_params`.

```julia
@testset "markersize attribute + default point_padding (#21)" begin
    using CairoMakie
    xs = [0.0, 1.0, 2.0]; ys = [0.0, 1.0, 0.5]
    labels = ["a", "b", "c"]

    # Default point_padding is now 5.0 on the recipe surface.
    fig = Figure(); ax = Axis(fig[1, 1])
    p = textrepel!(ax, xs, ys; text = labels)
    Makie.update_state_before_display!(fig)
    @test p.computed_params[].point_padding == 5.0

    # markersize sets effective point_padding = m/2 + 0.5.
    fig2 = Figure(); ax2 = Axis(fig2[1, 1])
    p2 = textrepel!(ax2, xs, ys; text = labels, markersize = 9)
    Makie.update_state_before_display!(fig2)
    @test p2.computed_params[].point_padding == 9 / 2 + 0.5   # == 5.0

    # markersize OVERRIDES an explicit point_padding.
    fig3 = Figure(); ax3 = Axis(fig3[1, 1])
    p3 = textrepel!(ax3, xs, ys; text = labels, markersize = 20, point_padding = 99.0)
    Makie.update_state_before_display!(fig3)
    @test p3.computed_params[].point_padding == 20 / 2 + 0.5  # markersize wins

    # A Vector markersize raises a clear error. The validation lives in the solve lift,
    # so force the lift to evaluate by dereferencing a computed node (don't rely on the
    # exception surfacing through update_state_before_display! alone — Makie may defer it).
    fig4 = Figure(); ax4 = Axis(fig4[1, 1])
    p4 = textrepel!(ax4, xs, ys; text = labels, markersize = [9, 9, 9])
    @test_throws Exception begin
        Makie.update_state_before_display!(fig4)
        p4.computed_params[]      # force the lift to run ⇒ ArgumentError propagates
    end
end
```

- [ ] **Step 2: Run to verify they fail**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-<agent-id>.log` then `grep -nA4 "markersize attribute" test/output/test-<agent-id>.log`
Expected: FAIL — `point_padding` defaults to `2.0`, `markersize` is an unknown attribute.

- [ ] **Step 3: Add the attribute, bump the default, derive in the lift**

In `src/recipe.jl`, in the `@recipe TextRepel` block, change the `point_padding` default and its docstring, and add the `markersize` attribute right after it:

```julia
    "Pixel marker-clearance: minimum gap from every scatter marker to the nearest label text edge, enforced after legalize (own and foreign markers). Also the connector anchor-end trim. Set to `marker_radius + small_gap`, or use `markersize`. (Under the in-tree ForceSolver it is the point-repulsion halo radius.)"
    point_padding = 5.0
    "Size of the SIBLING scatter marker, if any. `textrepel!` draws NO markers itself; this only tells the solver how much to clear. When set (scalar), overrides `point_padding` with `markersize/2 + 0.5`. Assumes a disc marker in `markerspace = :pixel` (the scatter default); for other markers set `point_padding` directly. `nothing` = use `point_padding`."
    markersize = nothing
```

In `Makie.plot!(p::TextRepel)`, make **four targeted edits** to the existing `solved = lift(...) do ...` block — do **not** retype the whole block (it carries a load-bearing multi-line comment about `bounds`/`computed_params` that must be preserved):

1. **Append `p.markersize`** as the last input to the `lift(...)` call (after `p.min_segment_length`).
2. **Append `ms`** as the last argument of the `do px, labels, ..., ml` parameter list.
3. **Insert the effective-`point_padding` derivation** immediately after the `sizes = measure_labels(labels, font, fs, 1.0)` line:

```julia
        # markersize (sibling scatter) overrides point_padding when set. textrepel!
        # draws no markers; this only declares the sibling size for clearance.
        eff_pp = if ms === nothing
            Float64(pp)
        elseif ms isa Real
            Float64(ms) / 2 + 0.5
        else
            throw(ArgumentError("textrepel!: `markersize` must be a scalar Real or nothing; got $(typeof(ms)). Per-point marker sizes are not supported — set `point_padding` directly."))
        end
```

4. **Swap** the `point_padding = Float64(pp)` argument in the `RepelParams(; ...)` construction to `point_padding = eff_pp`.

Leave everything else (the `bounds`/`computed_params` comment, the `solve_cluster` call, the returned NamedTuple) untouched. `computed_params` already exposes `params`, so `point_padding` now carries `eff_pp`.

- [ ] **Step 4: Run to verify they pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-<agent-id>.log` then `grep -nA4 "markersize attribute\|Test Summary" test/output/test-<agent-id>.log`
Expected: PASS for the new testset. Other integration/annotation tests may now fail from the default bump — Task 7 fixes those.

- [ ] **Step 5: Commit**

```bash
git add src/recipe.jl test/test_integration.jl
git commit -m "feat(recipe): markersize attribute; default point_padding 2.0→5.0 (#21)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 5: `TextRepelAlgorithm` ctor default + docstrings

**Files:**
- Modify: `src/annotation_algorithm.jl`
- Test: `test/test_annotation_algorithm.jl`

- [ ] **Step 1: Write the failing test** — append to `test/test_annotation_algorithm.jl`.

```julia
@testset "TextRepelAlgorithm default point_padding = 5.0 (#21)" begin
    alg = TextRepelAlgorithm()
    @test alg.params.point_padding == 5.0
    # explicit override still works
    alg2 = TextRepelAlgorithm(; point_padding = 1.5)
    @test alg2.params.point_padding == 1.5
end
```

- [ ] **Step 2: Run to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-<agent-id>.log` then `grep -nA3 "default point_padding = 5.0" test/output/test-<agent-id>.log`
Expected: FAIL — `alg.params.point_padding == 0.0` (inherited `RepelParams` default).

- [ ] **Step 3: Inject the default in the kwarg ctor**

In `src/annotation_algorithm.jl`, in the keyword constructor, change the `RepelParams` construction so a user value wins but the default is `5.0`:

```julia
    params = RepelParams(; merge((; point_padding = 5.0), values(kwargs))...)
```

(Replace the existing `params = RepelParams(; kwargs...)` line. `values(kwargs)` is the `NamedTuple` of forwarded keywords; `merge` lets a user-supplied `point_padding` override the `5.0` default. The explicit-`RepelParams` constructor `TextRepelAlgorithm(params::RepelParams)` is unchanged — it takes the params verbatim.)

Update the `TextRepelAlgorithm` docstring: note `point_padding` defaults to `5.0` (marker clearance) on this surface and is a generic anchor keep-out (clearance from the data point); add the pinned carve-out ("a pinned label whose offset covers its own marker is held bit-identically").

- [ ] **Step 4: Run to verify it passes**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-<agent-id>.log` then `grep -nA3 "default point_padding = 5.0\|Test Summary" test/output/test-<agent-id>.log`
Expected: PASS for the new testset.

- [ ] **Step 5: Commit**

```bash
git add src/annotation_algorithm.jl test/test_annotation_algorithm.jl
git commit -m "feat(annotation): TextRepelAlgorithm default point_padding=5.0 (#21)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 6: Hero dataset extraction + `point_overlaps == 0` integration test

**Files:**
- Modify: `examples/readme_example.jl` (extract shared dataset, add `markersize = 9`)
- Test: `test/test_integration.jl`

- [ ] **Step 1: Extract the hero dataset into a shared, deterministic constructor**

In `examples/readme_example.jl`, replace the inline data-generation block (the `knot_centers`/loops through `points = Point2f.(xs, ys)`) with a call to a small pure function defined at the top of the file, so both the example and the test build the identical scene:

```julia
# Deterministic hero dataset (shared with test/test_integration.jl). Uses the GLOBAL RNG
# via Random.seed! + bare randn() in the SAME draw order (x then y per point) as the
# original inline block, so the points — and the committed hero image — are byte-identical.
# (A self-contained MersenneTwister would produce DIFFERENT points and silently change the
# image + invalidate the documented point_overlaps==1 baseline — caught in plan review.)
function hero_dataset()
    Random.seed!(20260527)
    knot_centers = [(-0.7, 0.55), (0.75, -0.45), (0.05, 0.9)]
    knot_counts  = [3, 3, 2]
    xs = Float64[]; ys = Float64[]
    for (c, k) in zip(knot_centers, knot_counts), _ in 1:k
        push!(xs, c[1] + 0.03 * randn()); push!(ys, c[2] + 0.03 * randn())
    end
    for _ in 1:14
        push!(xs, 0.85 * randn()); push!(ys, 0.85 * randn())
    end
    labels = ["node $(i)" for i in 1:length(xs)]
    return xs, ys, labels
end
```

Keep `using Random` (the function uses `Random.seed!` + `randn`); remove the top-level
`Random.seed!(20260527)` line (the function now owns seeding). Set
`xs, ys, labels = hero_dataset()` and, immediately after, **retain** `points = Point2f.(xs, ys)`
(the `annotation!` call uses `points`). Pass `markersize = 9` to **both** the `textrepel!`
and the `annotation!`/`TextRepelAlgorithm` calls so the panels clear the markers:

```julia
    textrepel!(ax2, xs, ys; text = labels, fontsize = 13, markersize = 9)
    ...
    annotation!(ax3, points; text = labels, fontsize = 13,
                style = Makie.Ann.Styles.Line(),
                algorithm = TextRepelAlgorithm(; point_padding = 9 / 2 + 0.5))
```

- [ ] **Step 2: Write the failing integration test**

Append to `test/test_integration.jl`. Reconstruct the same dataset (copy the seeded constructor verbatim — tests must not `include` an example), solve, and assert. This validates the floor on the **hero dataset geometry** (it renders in a 400×400 fig, not the 1200×400 three-panel hero layout, so pixel anchors differ from the image — do not over-read it as pixel-identical to the committed PNG).

```julia
@testset "hero dataset geometry: no marker occlusion (#21)" begin
    using CairoMakie
    using Random              # Random.seed!, randn (global RNG)
    using MakieTextRepel: ProjectionSolver, RepelParams, solve_cluster, label_cost,
                          measure_labels

    function hero_dataset()
        Random.seed!(20260527)
        knot_centers = [(-0.7, 0.55), (0.75, -0.45), (0.05, 0.9)]
        knot_counts  = [3, 3, 2]
        xs = Float64[]; ys = Float64[]
        for (c, k) in zip(knot_centers, knot_counts), _ in 1:k
            push!(xs, c[1] + 0.03 * randn()); push!(ys, c[2] + 0.03 * randn())
        end
        for _ in 1:14
            push!(xs, 0.85 * randn()); push!(ys, 0.85 * randn())
        end
        labels = ["node $(i)" for i in 1:length(xs)]
        return xs, ys, labels
    end

    xs, ys, labels = hero_dataset()
    # Render once to get pixel anchors + sizes the way the recipe does.
    fig = Figure(size = (400, 400)); ax = Axis(fig[1, 1])
    p = textrepel!(ax, xs, ys; text = labels, fontsize = 13, markersize = 9)
    Makie.update_state_before_display!(fig)
    stats_dropped = count(p.computed_dropped[])
    # Recompute Q on the realised solve via label_cost.
    anchors = p.computed_anchors[]; sizes = p.computed_sizes[]
    offsets = p.computed_offsets[]; dropped = p.computed_dropped[]
    bnds = p.computed_params[].bounds
    q = label_cost(anchors, sizes; offsets = offsets, bounds = bnds, dropped = dropped,
                   box_padding = 4.0, point_padding = 9 / 2 + 0.5,
                   min_segment_length = 2.0)
    @test q.point_overlaps == 0

    # Drop baseline: re-derive by running this test once and recording the value below.
    # Frozen so point_overlaps==0 cannot be met by dropping the occluding label.
    HERO_DROP_BASELINE = 0   # <-- set to the observed count(dropped) on first green run
    @test stats_dropped ≤ HERO_DROP_BASELINE
end
```

- [ ] **Step 3: Run; record the baseline; re-run**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-<agent-id>.log` then `grep -nA6 "hero dataset" test/output/test-<agent-id>.log`
Expected: `q.point_overlaps == 0` passes. If the `stats_dropped ≤ HERO_DROP_BASELINE` line fails, read the actual `count(dropped)` from the failure, set `HERO_DROP_BASELINE` to that integer (it should be small, ideally 0), add a comment `# observed YYYY-MM-DD`, and re-run to green. If `point_overlaps` is **not** 0, the floor is not holding on the real hero scene — return to Task 2 (check `mc`, soft mask) before forcing the baseline.

- [ ] **Step 4: Commit**

```bash
git add examples/readme_example.jl test/test_integration.jl
git commit -m "test(integration): hero point_overlaps==0; share seeded dataset (#21)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 7: Default-bump fallout + stale-comment sweep + full suite green

**Files:**
- Modify: `test/test_integration.jl` ("connectors suppressed" fixture), `test/test_annotation_algorithm.jl` (warm-start comment + ctor sweep), and any layout-sensitive force/projection tests that shift.

- [ ] **Step 1: Run the full suite and enumerate every failure**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-<agent-id>.log` then `grep -nE "Test Summary|Fail|Error" test/output/test-<agent-id>.log` and `grep -nB1 -A6 -i "fail\|error" test/output/test-<agent-id>.log`
Expected: a list of testsets that shifted under `point_padding = 5.0` + point-aware legalize. Work through each below.

- [ ] **Step 2: Re-derive the "connectors suppressed when clamp pins anchor inside its own label" fixture**

Find it: `grep -n "suppressed" test/test_integration.jl`. This test was written for the old behaviour (no own-anchor keep-out, `point_padding = 2.0`) and its rationale comment cites `point_padding = 2.0`. The own-anchor keep-out now tries to push the label off its own anchor.

Decide empirically: run that fixture's solve in the REPL
(`julia --project=. -i -e 'using MakieTextRepel, CairoMakie'`, reconstruct the fixture's anchors/sizes/bounds, call `solve_cluster`), inspect whether the anchor ends up inside or outside the label box:
- If the viewport is too small to clear the anchor (over-capacity ⇒ floor waived ⇒ anchor still inside): the connector stays suppressed — keep the assertion, **fix the stale comment** to explain it survives because the scene is over-capacity for the new 5px floor (not because `point_padding = 2.0`).
- If the anchor is now pushed clear (connector drawn): update the assertion to expect the connector, and rewrite the comment to describe the new clear-the-marker behaviour.

Document the decided outcome in the test comment.

- [ ] **Step 3: Fix `test_annotation_algorithm.jl` comment + sweep ctor sites**

Find the warm-start invariant test: `grep -n "point_padding" test/test_annotation_algorithm.jl`. Correct any comment asserting "default point_padding = 0" to reflect the new `5.0` default + 5px clearance. Then sweep all bare `TextRepelAlgorithm()` / kwarg-ctor call sites in the file: each now solves with `point_padding = 5.0` + point-aware legalize. For each, confirm its assertions (inequalities, in-bounds, offset-direction) still hold against the new log output; where an assertion encoded the old geometry, pin that ctor call to an explicit `point_padding = 0.0` (preserving its intent) **or** update the expected value. Do not weaken a real invariant — prefer pinning `point_padding` to keep the test's original geometry.

- [ ] **Step 4: Fix any shifted force/projection layout tests**

For each remaining failure, determine if it is a numeric value that legitimately shifted (e.g. a frozen leader/offset constant) or a real regression:
- Legitimate shift: re-derive the constant from the new log output and update it, with a comment noting it changed under the #21 default bump. (The ForceSolver default is unchanged at `0.0`, so pure force-model tests should be unaffected; failures there indicate a real regression — investigate, do not blindly update.)
- Real regression: stop and diagnose; the keep-out or default change has a bug.

- [ ] **Step 5: Run the full suite to green**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-<agent-id>.log` then `grep -nE "Test Summary|Fail|Error" test/output/test-<agent-id>.log`
Expected: all testsets pass; `0 failed, 0 errored` across the suite.

- [ ] **Step 6: Commit**

```bash
git add test/
git commit -m "test: fix default-bump fallout + stale comments under point-aware legalize (#21)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 8: Docs (params, CLAUDE.md) + regenerate hero image

**Files:**
- Modify: `src/params.jl` (docstring), `CLAUDE.md`
- Regenerate: `assets/example.png`

- [ ] **Step 1: Reframe the `RepelParams.point_padding` docstring**

In `src/params.jl`, update the inline comment/docstring for `point_padding` to read (default stays `0.0`):

```julia
    point_padding::Float64          = 0.0   # marker clearance: gap from each anchor/marker to the nearest label text edge (enforced post-legalize by the ProjectionSolver as a keep-out). Doubles as the ForceSolver point-repulsion halo radius. Primitive default 0.0; user surfaces (recipe attr, TextRepelAlgorithm) default to 5.0.
```

- [ ] **Step 2: Update CLAUDE.md**

In `CLAUDE.md`:
- In the `src/legalize.jl` architecture bullet, add that `legalize` accepts a `soft` node class (participates in projection, excluded from the returned `residual`) used for marker keep-outs.
- In the `src/solvers/projection.jl` bullet, add that `legalize_and_drop` appends every anchor as a fixed+soft keep-out node (half-extent `point_padding − box_padding`), enforcing the marker-clearance floor for own and foreign markers without ever dropping a label to satisfy it.
- In "Project state & history", move "point-aware legalize" from the deferred non-goal to implemented (issue #21), and note `point_padding` is now the marker-clearance knob with a `5.0` default on the user surfaces (primitive stays `0.0`), plus the recipe `markersize` convenience attribute.
- In the `src/recipe.jl` bullet, note the new `markersize` attribute and the `point_padding` reframing.

- [ ] **Step 3: Regenerate the hero image**

Run: `julia --project=. examples/readme_example.jl`
Expected: prints `wrote .../assets/example.png`. Open `assets/example.png` and visually confirm no label text is occluded by a marker disc in the middle/right panels.

- [ ] **Step 4: Commit**

```bash
git add src/params.jl CLAUDE.md assets/example.png
git commit -m "docs: reframe point_padding as marker clearance; regenerate hero (#21)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 9: Final verification + PR

- [ ] **Step 1: Full suite, clean run**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-<agent-id>.log` then `grep -nE "Test Summary|Fail|Error" test/output/test-<agent-id>.log`
Expected: every testset passes, `0 failed, 0 errored`.

- [ ] **Step 2: Confirm acceptance criteria**

- [ ] Floor regression (Task 2) green — non-pinned/non-dropped labels clear own + foreign markers post-legalize.
- [ ] Hero `point_overlaps == 0` with `dropped ≤ baseline` (Task 6) green.
- [ ] Anti-cascade (Task 3) green — marker clearance never drops a label.
- [ ] Regenerated `assets/example.png` shows no marker-occluded labels.

- [ ] **Step 3: Open the PR**

```bash
git push -u origin feat/marker-clearance-floor
gh pr create --title "Marker-clearance floor: point-aware legalize (#21)" \
  --body "Closes #21. Adds anchor keep-out nodes to legalize (soft: they push but never trigger a drop), so labels clear scatter markers post-legalize. point_padding reframed as the marker-clearance knob (default 5.0 on user surfaces; RepelParams primitive stays 0.0 for the ForceSolver halo). New recipe markersize convenience attribute. Hero point_overlaps 1→0.

🤖 Generated with [Claude Code](https://claude.com/claude-code)"
```

---

## Self-review notes

- **Spec coverage:** §1 (Task 2), §1a soft nodes (Task 1, verified Task 3), §2 markersize (Task 4), §3 default decoupling (Tasks 4 recipe + 5 annotation; primitive untouched Task 8), §4 annotation semantics (Task 5 docstring), §5 floor scope (Tasks 2/3 tests), §6 docs (Task 8), §7 perf (no code, noted in CLAUDE.md/spec). Out-of-scope snooping: not implemented (correct). All Testing items 1–7 mapped to Tasks 1–8.
- **Type consistency:** `legalize(...; fixed, soft, only_move, rounds)` signature is consistent between Task 1 (definition) and Task 2 (call site). `mc = Float32(point_padding − box_padding)`; keep-out node `psize = Vec2f(2mc, 2mc)`. `eff_pp` derivation (`ms/2 + 0.5`) consistent between recipe (Task 4) and annotation (`9/2+0.5`, Task 5/6). `point_covered(p, box, pad)` argument order matches `geometry.jl`.
- **Known measure-then-record steps (not placeholders):** `HERO_DROP_BASELINE` (Task 6 Step 3) and shifted frozen constants (Task 7 Step 4) are values that must be read from a green/failing run and recorded — the mechanism is fully specified; only the integers are environment-derived.
