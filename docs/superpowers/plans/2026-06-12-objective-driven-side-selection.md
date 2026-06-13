# Objective-Driven Side Selection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the ProjectionSolver optimize the placement quality it already measures — labels dodge scatter markers, prefer readable sides, and end up crossing-free — by enriching `side_select`'s objective into a lexicographic key and adding a swap-based local search that drives crossings to zero.

**Architecture:** Three staged, independently-shippable increments on the existing pure pipeline. Stage 1 adds a shared `point_covered` predicate and folds marker overlaps into a lexicographic side-select key (replacing the weighted scalar) plus a `point_overlaps` Q component. Stage 2 adds a gentle Imhof side-preference to the soft tier of that key. Stage 3 adds a crossing term to the best-of-passes selector and a post-legalize swap-to-fixpoint local search whose accept/reject gate is the read-only `label_cost` Q itself — literally optimizing what we measure.

**Tech Stack:** Julia 1.11, GeometryBasics (pure layers), Makie 0.24 (recipe/annotation surfaces only), Test + CairoMakie (test target).

---

## Testing note (read before running anything)

Julia precompiles on every `Pkg.test()`, costing minutes. Run the suite once, tee to an agent-scoped log under `test/output/` (gitignored), then `grep` the log. Pick one slug for your session.

```bash
LOG="test/output/test-objdrive.log"          # replace slug with your agent/job id
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -E "Test Summary|Fail|Error|Pass" "$LOG"
```

Re-run the full suite only after a code change. For a quick pure-layer check without `Pkg.test()`:

```bash
julia --project=. -i -e 'using MakieTextRepel'   # then call MakieTextRepel.<fn> in the REPL
```

## File structure (what each task touches)

- `src/geometry.jl` — add `point_covered` (shared point-in-padded-box predicate). Stage 1.
- `src/side_select.jl` — lexicographic key; marker term (Stage 1); side-preference term (Stage 2); crossing term in `global_cost` (Stage 3).
- `src/cost.jl` — add `point_overlaps` Q component via `point_covered`. Stage 1.
- `src/solvers/projection.jl` — `ProjectionStats` shape (Stage 1); extract `legalize_and_drop` helper + swap-to-fixpoint local search (Stage 3).
- `src/annotation_algorithm.jl` — all-pinned bypass literal + two docstrings (Stage 1).
- `src/MakieTextRepel.jl` — export nothing new; `point_covered` is internal (no change expected, but confirm `geometry.jl` symbols are in scope where used — they are, same module).
- Tests: `test/test_geometry.jl`, `test/test_side_select.jl`, `test/test_cost.jl`, `test/test_annotation_algorithm.jl`, `test/test_projection_solver.jl`.
- `examples/readme_example.jl` / `assets/example.png` — regenerate at the end (Stage 3).

---

# Stage 1 — Marker / point avoidance

## Task 1.1: `point_covered` predicate

**Files:**
- Modify: `src/geometry.jl` (add after `point_push`, ~line 45)
- Test: `test/test_geometry.jl`

- [ ] **Step 1: Write the failing test**

Add to `test/test_geometry.jl` (inside the file's top-level `@testset` block, or as a new `@testset`):

```julia
@testset "point_covered" begin
    using MakieTextRepel: point_covered, box_at
    box = box_at(Point2f(0, 0), Vec2f(0, 0), Vec2f(10, 10))  # [-5,5]×[-5,5]
    # strictly inside → covered (no padding)
    @test point_covered(Point2f(0, 0), box, 0.0)
    # outside the bare box but inside the padded halo → covered
    @test point_covered(Point2f(6, 0), box, 2.0)        # x=6 < 5+2
    @test !point_covered(Point2f(8, 0), box, 2.0)       # x=8 > 5+2
    # exactly on the expanded edge → NOT covered (strict, matches clip_to_box_edge)
    @test !point_covered(Point2f(7, 0), box, 2.0)       # x == 5+2
    # corner halo respected on both axes
    @test point_covered(Point2f(6, 6), box, 2.0)
    @test !point_covered(Point2f(6, 8), box, 2.0)
end
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-objdrive.log; grep -nA2 "point_covered" test/output/test-objdrive.log`
Expected: FAIL — `UndefVarError: point_covered not defined`.

- [ ] **Step 3: Implement `point_covered`**

Add to `src/geometry.jl`:

```julia
"""
True iff point `p` lies strictly inside `box` expanded by `padding` on every side.
Strict inequalities (a point on the expanded edge is not covered) match the
`clip_to_box_edge` convention. Shared by `side_select`'s marker-avoidance term and
`label_cost`'s `point_overlaps` count, so the engine objective and the Q scoreboard
count the same events.
"""
function point_covered(p::Point2f, box::Rect2f, padding::Real)
    pad = Float32(padding)
    lo = box.origin .- pad
    hi = box.origin .+ box.widths .+ pad
    return p[1] > lo[1] && p[1] < hi[1] && p[2] > lo[2] && p[2] < hi[2]
end
```

- [ ] **Step 4: Run test to verify it passes**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-objdrive.log; grep -E "Test Summary|Fail|Error" test/output/test-objdrive.log`
Expected: PASS, no failures/errors.

- [ ] **Step 5: Commit**

```bash
git add src/geometry.jl test/test_geometry.jl
git commit -m "feat(geometry): add point_covered shared point-in-padded-box predicate"
```

---

## Task 1.2: Lexicographic side-select key + marker-avoidance term

This replaces `side_select`'s weighted scalar (`ov*overlap_weight + ‖offset‖`) with a
lexicographic tuple `(hard_overlaps, soft)` so overlap-avoidance *provably* dominates
leader length, and folds label–marker overlaps into `hard_overlaps` at the same weight
as label–label overlaps (the decided `W_pt = W_lap`). Markers are the foreign anchors;
a label never avoids its own anchor.

**Files:**
- Modify: `src/side_select.jl` (the `global_cost` closure `:76-89`, the per-slot greedy `:96-112`, docstring `:1-18`, signature `:19-25`)
- Test: `test/test_side_select.jl`

- [ ] **Step 1: Write the failing tests**

Add to `test/test_side_select.jl`:

```julia
@testset "side_select steers a label off a foreign marker" begin
    # Two anchors. Label 1 is large; its default short-leader slot would cover anchor 2's
    # marker. With the marker term, label 1 must pick a slot that clears anchor 2.
    anchors = [Point2f(100, 100), Point2f(140, 100)]
    sizes   = [Vec2f(60, 20), Vec2f(20, 10)]
    p       = RepelParams(box_padding = 0.0, point_padding = 2.0)
    ps      = [sizes[i] .+ 2 * Float32(p.box_padding) for i in 1:2]
    bounds  = Rect2f(0, 0, 400, 400)
    seed    = [Vec2f(0, 0), Vec2f(0, 0)]
    sel = side_select(anchors, sizes, ps, bounds, seed, p)
    # anchor 2 must NOT be covered by label 1's chosen box (unpadded text box + point_padding)
    box1 = box_at(anchors[1], sel[1], sizes[1])
    @test !MakieTextRepel.point_covered(anchors[2], box1, p.point_padding)
end

@testset "side_select does NOT avoid a label's own anchor" begin
    # A single label: its own anchor is inside/adjacent to its box by construction.
    # The marker term must skip j == i, so a lone label still takes its shortest slot.
    anchors = [Point2f(200, 200)]
    sizes   = [Vec2f(40, 16)]
    p       = RepelParams(box_padding = 0.0, point_padding = 2.0)
    ps      = [sizes[1] .+ 2 * Float32(p.box_padding)]
    bounds  = Rect2f(0, 0, 400, 400)
    seed    = [Vec2f(0, 0)]
    sel = side_select(anchors, sizes, ps, bounds, seed, p)
    # TR slot (shortest-leader, most-preferred) is chosen — own anchor not treated as obstacle
    @test sel[1] == MakieTextRepel._constrain(
        MakieTextRepel.slot_offset(:TR, sizes[1], p.point_padding), p.only_move)
end
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-objdrive.log; grep -nA3 "own anchor\|foreign marker" test/output/test-objdrive.log`
Expected: FAIL — label 1 still covers anchor 2 (marker term not yet present).

- [ ] **Step 3: Implement the lexicographic key + marker term**

In `src/side_select.jl`, replace the `global_cost` closure (lines 76-89) with a version
returning a lexicographic tuple `(hard_overlaps::Int, soft::Float64)` and counting marker
overlaps:

```julia
    # Lexicographic arrangement key: (hard_overlaps, soft). hard_overlaps = label–label
    # overlap pairs + label–marker point overlaps (W_pt = W_lap: same lex level). soft =
    # total leader length (the tiebreak). Compared with Julia tuple `<` (lexicographic),
    # so overlap-avoidance provably dominates leader length regardless of pixel scale.
    function global_key(s)
        hard = 0
        soft = 0.0
        for i in 1:n
            b   = box_at(anchors[i], s[i], psizes[i])          # box-padded, for label–label
            bm  = box_at(anchors[i], s[i], sizes[i])           # unpadded text box, for markers
            for j in (i+1):n
                (overlap_push(b, box_at(anchors[j], s[j], psizes[j])) != Vec2f(0, 0)) && (hard += 1)
            end
            for ob in obstacles
                (overlap_push(b, ob) != Vec2f(0, 0)) && (hard += 1)
            end
            for j in 1:n                                       # foreign markers (own anchor skipped)
                j == i && continue
                point_covered(anchors[j], bm, p) && (hard += 1)
            end
            soft += sqrt(Float64(s[i][1])^2 + Float64(s[i][2])^2)
        end
        return (hard, soft)
    end
```

Note `p` is already `params.point_padding` (line 33). Then replace the best-of-passes
bookkeeping (lines 91, 113-114) to compare keys with `<`:

```julia
    best_sel = copy(sel); best_key = global_key(sel)
```

and inside the pass loop, replacing lines 113-114:

```julia
        gk = global_key(sel)
        if gk < best_key; best_key = gk; best_sel = copy(sel); end
```

Replace the per-slot greedy cost (lines 96-111) so each slot's key is `(ov, soft)`:

```julia
            besto = sel[i]; bestkey = (typemax(Int), Inf)
            for o in cands[i]
                b  = box_at(anchors[i], o, psizes[i])
                bm = box_at(anchors[i], o, sizes[i])
                ov = 0
                for j in 1:n
                    j == i && continue
                    (overlap_push(b, box_at(anchors[j], sel[j], psizes[j])) != Vec2f(0, 0)) && (ov += 1)
                    point_covered(anchors[j], bm, p) && (ov += 1)   # foreign marker term
                end
                for ob in obstacles
                    (overlap_push(b, ob) != Vec2f(0, 0)) && (ov += 1)
                end
                soft = sqrt(Float64(o[1])^2 + Float64(o[2])^2)
                key = (ov, soft)
                if key < bestkey; bestkey = key; besto = o; end
            end
            (besto != sel[i]) && (changed = true)
            sel[i] = besto
```

Remove the now-unused `overlap_weight` kwarg from the signature (line 25) and docstring
(lines 6, 11-13). New signature line:

```julia
                     obstacles::Vector{Rect2f}           = Rect2f[],
                     passes::Int = 6)
```

Update the docstring header comment (lines 5-8) to describe the lexicographic key:

```julia
# Each label's candidate offsets are its in-bounds Imhof slots (constrained by
# only_move). Seeded from the Voronoi-informed init, then refined by index-ordered
# greedy sweeps minimizing the lexicographic key
#   (hard_overlaps, leader_length)
# where hard_overlaps counts label–label box overlaps, label–obstacle overlaps, AND
# label–marker point overlaps (foreign anchors covered by the label box). Overlap
# avoidance provably dominates leader length. Pinned labels are fixed; obstacles are
# fixed boxes; a label never avoids its own anchor. Deterministic.
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-objdrive.log; grep -E "Test Summary|Fail|Error" test/output/test-objdrive.log`
Expected: PASS. The existing `side_select` tests (shortest-leader, head-on, pinned, obstacle, only_move, deterministic) must still pass — the lexicographic key is behavior-equivalent to the old scalar when there are no markers in the way.

- [ ] **Step 5: Commit**

```bash
git add src/side_select.jl test/test_side_select.jl
git commit -m "feat(side_select): lexicographic key + label–marker avoidance term"
```

---

## Task 1.3: `point_overlaps` Q component in `label_cost`

**Files:**
- Modify: `src/cost.jl` (docstring `:4-6`, signature/body)
- Test: `test/test_cost.jl`

- [ ] **Step 1: Write the failing test**

Add to `test/test_cost.jl`:

```julia
@testset "label_cost point_overlaps" begin
    # Two anchors 30px apart on x. Label 1 (wide) at offset 0 covers anchor 2.
    anchors = [Point2f(100, 100), Point2f(130, 100)]
    sizes   = [Vec2f(80, 20), Vec2f(10, 10)]
    offs    = [Vec2f(0, 0), Vec2f(0, 40)]      # label 1 centered on its anchor → covers anchor 2
    bounds  = Rect2f(0, 0, 400, 400)
    q = label_cost(anchors, sizes; offsets = offs, bounds = bounds,
                   box_padding = 0.0, point_padding = 2.0, min_segment_length = 4.0)
    @test q.point_overlaps == 1                # anchor 2 sits under label 1's box
    # move label 1 far up so it no longer covers anchor 2
    offs2 = [Vec2f(0, 80), Vec2f(0, 40)]
    q2 = label_cost(anchors, sizes; offsets = offs2, bounds = bounds,
                    box_padding = 0.0, point_padding = 2.0, min_segment_length = 4.0)
    @test q2.point_overlaps == 0
end
```

- [ ] **Step 2: Run test to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-objdrive.log; grep -nA3 "point_overlaps" test/output/test-objdrive.log`
Expected: FAIL — `q` has no field `point_overlaps`.

- [ ] **Step 3: Implement `point_overlaps`**

In `src/cost.jl`, after the `overlaps` loop (the `for i in 1:n, j in (i+1):n` block ending
at line 44) and before the `active`/`mean_leader` block, add:

```julia
    point_overlaps = 0
    for i in 1:n
        isdrp(i) && continue
        bm = Rect2f(Point2f(cx[i] - Float64(sizes[i][1]) / 2, cy[i] - Float64(sizes[i][2]) / 2),
                    Vec2f(sizes[i]))                              # unpadded text box of label i
        for j in 1:n
            (j == i || isdrp(j)) && continue
            point_covered(anchors[j], bm, point_padding) && (point_overlaps += 1)
        end
    end
```

Update the returned tuple (line ~52) to include it:

```julia
    return (; overlaps = overlaps, point_overlaps = point_overlaps,
              mean_leader = mean_leader, crossings = crossings)
```

Update the docstring signature line (`cost.jl:4-6`) and the bullet list to mention
`point_overlaps` (count of non-dropped labels covering a non-dropped foreign anchor, via
the shared `point_covered`, using `point_padding`).

- [ ] **Step 4: Run test to verify it passes**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-objdrive.log; grep -E "Test Summary|Fail|Error" test/output/test-objdrive.log`
Expected: PASS. Existing `test_cost.jl` assertions use named field access (`q.overlaps` etc.), so the new field doesn't break them.

- [ ] **Step 5: Commit**

```bash
git add src/cost.jl test/test_cost.jl
git commit -m "feat(cost): add point_overlaps Q component via shared point_covered"
```

---

## Task 1.4: Extend `ProjectionStats` shape to carry `point_overlaps`

The `solve_stats` tuple gains `point_overlaps`. This touches six runtime/doc sites and
two tests; two of the sites are literal tuples that hard-fail if missed.

**Files:**
- Modify: `src/solvers/projection.jl` (`:9-10` type alias, `:13-15` docstring, `:22-24` constructor literal, `:147-148` writeback)
- Modify: `src/annotation_algorithm.jl` (`:33-34` and `:80` docstrings, `:125-126` bypass literal)
- Test: `test/test_annotation_algorithm.jl` (`:52-53` canary, `:118-119` all-pinned equality)

- [ ] **Step 1: Update the two tests first (they encode the contract)**

In `test/test_annotation_algorithm.jl` line 52-53, add `:point_overlaps` to the expected set:

```julia
    @test Set(propertynames(s)) ==
          Set((:iter, :residual, :overlaps, :point_overlaps, :mean_leader, :crossings, :dropped))
```

And line 118-119, the all-pinned exact-equality assertion:

```julia
    @test solve_stats(alg) == (; overlaps = 0, point_overlaps = 0, mean_leader = 0f0,
                                 crossings = 0, iter = 0, residual = 0f0, dropped = 0)
```

- [ ] **Step 2: Run to verify they fail**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-objdrive.log; grep -nA3 "solve_stats" test/output/test-objdrive.log`
Expected: FAIL — current tuple lacks `point_overlaps`.

- [ ] **Step 3: Update all six source sites**

`src/solvers/projection.jl:9-10` — the typed alias (names tuple + element types):

```julia
const ProjectionStats = NamedTuple{(:overlaps, :point_overlaps, :mean_leader, :crossings, :iter, :residual, :dropped),
                                   Tuple{Int, Int, Float32, Int, Int, Float32, Int}}
```

`src/solvers/projection.jl:13-15` — docstring field list: add `point_overlaps` after `overlaps`.

`src/solvers/projection.jl:22-24` — constructor zero-init literal:

```julia
ProjectionSolver(params::RepelParams) =
    ProjectionSolver(params, Ref{ProjectionStats}((; overlaps = 0, point_overlaps = 0,
                                                     mean_leader = 0f0, crossings = 0,
                                                     iter = 0, residual = 0f0, dropped = 0)))
```

`src/solvers/projection.jl:147-148` — the real writeback:

```julia
    s.stats[] = (; overlaps = q.overlaps, point_overlaps = q.point_overlaps,
                   mean_leader = q.mean_leader, crossings = q.crossings,
                   iter = lz.rounds_used, residual = lz.residual, dropped = count(dropped))
```

`src/annotation_algorithm.jl:125-126` — the all-pinned bypass literal:

```julia
        alg.solver.stats[] = (; overlaps = 0, point_overlaps = 0, mean_leader = 0f0,
                                crossings = 0, iter = 0, residual = 0f0, dropped = 0)
```

(Keep the existing field order on that line consistent with the rest of the literal; the
NamedTuple is keyword-constructed so order is not load-bearing, but match for readability.)

`src/annotation_algorithm.jl:33-34` and `:80` — docstrings listing the tuple: add
`point_overlaps`.

- [ ] **Step 4: Run to verify pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-objdrive.log; grep -E "Test Summary|Fail|Error" test/output/test-objdrive.log`
Expected: PASS (whole suite — the shape change is consistent across all sites).

- [ ] **Step 5: Commit**

```bash
git add src/solvers/projection.jl src/annotation_algorithm.jl test/test_annotation_algorithm.jl
git commit -m "feat(stats): carry point_overlaps through ProjectionStats/solve_stats"
```

---

## Task 1.5: Multi-fixture Q battery (aggregate non-regression + post-legalize markers)

A committed scoreboard `@testset` over the existing fixture battery, asserting the solved
layout is overlap-free **and marker-free post-legalize**, with baseline Q recorded. This
catches the risk that point-unaware `legalize` re-covers a marker after Stage 1's discrete
win.

**Files:**
- Test: `test/test_projection_solver.jl` (new `@testset` near the existing aggregate one at `:27`)

- [ ] **Step 1: Write the battery test**

Add to `test/test_projection_solver.jl` (reuse the fixture shape from the existing
`:33-42` block — anchors/bounds/sizes per scene):

```julia
@testset "Q battery: marker-free and overlap-free post-legalize" begin
    using MakieTextRepel: ProjectionSolver, solve_cluster, label_cost, RepelParams
    fixtures = [
        ("sparse",  Rect2f(0, 0, 400, 400), [Point2f(80 + 60i, 200 + 20*(-1)^i) for i in 1:4],
                    [Vec2f(50, 18) for _ in 1:4]),
        ("knot",    Rect2f(0, 0, 300, 300), [Point2f(150 + 4randn_seed(i), 150 + 4randn_seed(i+10)) for i in 1:5],
                    [Vec2f(46, 16) for _ in 1:5]),
        ("collin",  Rect2f(0, 0, 500, 120), [Point2f(60 + 70i, 60) for i in 1:6],
                    [Vec2f(48, 16) for _ in 1:6]),
    ]
    total_overlaps = 0; total_point = 0
    for (name, bounds, anchors, sizes) in fixtures
        params = RepelParams(box_padding = 4.0, point_padding = 2.0, min_segment_length = 4.0)
        offs, dropped, _, _ = solve_cluster(ProjectionSolver(params), anchors, sizes, bounds)
        q = label_cost(anchors, sizes; offsets = offs, bounds = bounds, dropped = dropped,
                       box_padding = params.box_padding, point_padding = params.point_padding,
                       min_segment_length = params.min_segment_length)
        @test q.overlaps == 0                       # zero-overlap guarantee (feasible fixtures)
        @test q.point_overlaps == 0                 # markers cleared AFTER legalize
        total_overlaps += q.overlaps; total_point += q.point_overlaps
    end
    @test total_overlaps == 0 && total_point == 0   # aggregate non-regression baseline
end
```

If a `randn_seed` helper is not already defined at the top of the file, add a tiny
deterministic pseudo-noise helper near the top (after the file's leading comment):

```julia
# Deterministic per-index jitter (no RNG) so the knot fixture is reproducible.
randn_seed(i::Int) = Float32(sin(12.9898 * i) * 43758.5453 % 1.0)
```

- [ ] **Step 2: Run to verify it passes (or surfaces a real gap)**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-objdrive.log; grep -nA3 "Q battery" test/output/test-objdrive.log`
Expected: PASS. If `q.point_overlaps > 0` on the `knot` fixture, that is the legalize-erasure
risk materializing — STOP and escalate to the deferred point-aware-legalize follow-up
(see spec Non-goals) rather than weakening the assertion.

- [ ] **Step 3: Commit**

```bash
git add test/test_projection_solver.jl
git commit -m "test(projection): Q battery — marker-free + overlap-free post-legalize"
```

---

# Stage 2 — Side readability

## Task 2.1: Imhof side-preference in the soft tier

Add `W_side · rank(slot)` to the soft component of `side_select`'s key, where `rank` is the
slot's index in `IMHOF_ORDER` (TR=0 … TL=7). Lives strictly below `hard_overlaps`, so it
only decides among equal-overlap slots; a deliberate readability-for-leader trade of up to
`7·W_side` px.

**Files:**
- Modify: `src/side_select.jl` (candidate construction to track rank; per-slot key; `global_key`)
- Test: `test/test_side_select.jl`

- [ ] **Step 1: Write the failing test**

```julia
@testset "side_select prefers the readable (Imhof) side among equal-overlap slots" begin
    # A lone label with no conflicts: every in-bounds slot has hard_overlaps = 0, so the
    # side term decides. The most-preferred slot (TR, rank 0) must win even though several
    # slots have equal/near-equal leader length.
    anchors = [Point2f(200, 200)]
    sizes   = [Vec2f(40, 16)]
    p       = RepelParams(box_padding = 0.0, point_padding = 2.0)
    ps      = [sizes[1] .+ 2 * Float32(p.box_padding)]
    bounds  = Rect2f(0, 0, 400, 400)
    seed    = [Vec2f(-100, -100)]                  # seed nearest to BL, to prove the term overrides the seed
    sel = side_select(anchors, sizes, ps, bounds, seed, p)
    @test sel[1] == MakieTextRepel._constrain(
        MakieTextRepel.slot_offset(:TR, sizes[1], p.point_padding), p.only_move)
end

@testset "side_select: readability never overrides overlap avoidance" begin
    # Head-on conflict: the side term must not pull both labels onto TR and re-create overlap.
    anchors = [Point2f(195, 200), Point2f(205, 200)]
    sizes   = [Vec2f(40, 16), Vec2f(40, 16)]
    p       = RepelParams(box_padding = 0.0, point_padding = 2.0)
    ps      = [sizes[i] .+ 2 * Float32(p.box_padding) for i in 1:2]
    bounds  = Rect2f(0, 0, 400, 400)
    seed    = [Vec2f(0, 0), Vec2f(0, 0)]
    sel = side_select(anchors, sizes, ps, bounds, seed, p)
    b1 = box_at(anchors[1], sel[1], ps[1]); b2 = box_at(anchors[2], sel[2], ps[2])
    @test overlap_push(b1, b2) == Vec2f(0, 0)      # still separated despite the readability pull
end
```

- [ ] **Step 2: Run to verify the first test fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-objdrive.log; grep -nA3 "readable (Imhof)" test/output/test-objdrive.log`
Expected: FAIL — without the side term, the seed-nearest slot (BL) is chosen, not TR.

- [ ] **Step 3: Implement the side term**

In `src/side_select.jl`, add the weight constant near the top of the function body (after
`p = params.point_padding`, line 33):

```julia
    W_side = 1.5    # px per Imhof rank step; soft readability bias, below all hard terms
```

Track each candidate slot's rank alongside its offset. Change the candidate build (lines
41-51) so `cands[i]` is a `Vector{Tuple{Vec2f,Int}}` of `(offset, rank)`:

```julia
    cands = Vector{Vector{Tuple{Vec2f,Int}}}(undef, n)
    for i in 1:n
        cs = Tuple{Vec2f,Int}[]
        for (rank, s) in enumerate(IMHOF_ORDER)
            o = _constrain(slot_offset(s, sizes[i], p), params.only_move)
            inb(box_at(anchors[i], o, psizes[i])) && push!(cs, (o, rank - 1))
        end
        isempty(cs) && (cs = [(_constrain(slot_offset(s, sizes[i], p), params.only_move), rank - 1)
                              for (rank, s) in enumerate(IMHOF_ORDER)])
        cands[i] = cs
    end
```

Update the seed-nearest initial selection (lines 61-66) to unpack the tuple:

```julia
            best = cands[i][1][1]; bestd = Inf
            for (o, _) in cands[i]
                d = (Float64(o[1]) - seed[i][1])^2 + (Float64(o[2]) - seed[i][2])^2
                if d < bestd; bestd = d; best = o; end
            end
            sel[i] = best
```

`global_key` must include each label's chosen rank, or the best-of-passes selector ties TR
against BL (equal leader length) and keeps the seed arrangement instead of the readable one.
Track a parallel `sel_rank::Vector{Int}` alongside `sel`. Declare it next to `sel` (replacing
the bare `sel = Vector{Vec2f}(undef, n)` with both):

```julia
    sel = Vector{Vec2f}(undef, n)
    sel_rank = zeros(Int, n)            # Imhof rank of each label's currently-selected slot
```

In the initial selection loop (lines 57-68), record the chosen slot's rank for non-pinned
labels (pinned keep rank 0 — their offset is fixed and never re-scored against a slot):

```julia
        if isfixed(i)
            sel[i] = pinned_offsets[i]
        else
            best = cands[i][1][1]; bestrank = cands[i][1][2]; bestd = Inf
            for (o, rank) in cands[i]
                d = (Float64(o[1]) - seed[i][1])^2 + (Float64(o[2]) - seed[i][2])^2
                if d < bestd; bestd = d; best = o; bestrank = rank; end
            end
            sel[i] = best; sel_rank[i] = bestrank
        end
```

In the per-slot greedy loop, iterate `(o, rank)`, fold rank into `soft`, and write back the
winning rank:

```julia
            besto = sel[i]; bestrank = sel_rank[i]; bestkey = (typemax(Int), Inf)
            for (o, rank) in cands[i]
                b  = box_at(anchors[i], o, psizes[i])
                bm = box_at(anchors[i], o, sizes[i])
                ov = 0
                for j in 1:n
                    j == i && continue
                    (overlap_push(b, box_at(anchors[j], sel[j], psizes[j])) != Vec2f(0, 0)) && (ov += 1)
                    point_covered(anchors[j], bm, p) && (ov += 1)
                end
                for ob in obstacles
                    (overlap_push(b, ob) != Vec2f(0, 0)) && (ov += 1)
                end
                soft = sqrt(Float64(o[1])^2 + Float64(o[2])^2) + W_side * rank
                key = (ov, soft)
                if key < bestkey; bestkey = key; besto = o; bestrank = rank; end
            end
            (besto != sel[i]) && (changed = true)
            sel[i] = besto; sel_rank[i] = bestrank
```

Then in `global_key`, fold the selected rank into each label's `soft` contribution (this is
the same nested closure, so `sel_rank` is captured):

```julia
            soft += sqrt(Float64(s[i][1])^2 + Float64(s[i][2])^2) + W_side * sel_rank[i]
```

- [ ] **Step 4: Run to verify both tests pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-objdrive.log; grep -E "Test Summary|Fail|Error" test/output/test-objdrive.log`
Expected: PASS, including the head-on test (overlap avoidance still wins) and all prior
`side_select` tests. The "shortest-leader when unconflicted" test at `:12` may now prefer a
higher-Imhof slot at equal leader — verify it still holds; if it asserted a specific slot
that ties on leader, the more-preferred slot is the correct new answer (update that
assertion to the Imhof-preferred slot if needed, noting why).

- [ ] **Step 5: Commit**

```bash
git add src/side_select.jl test/test_side_select.jl
git commit -m "feat(side_select): gentle Imhof side-preference in the soft tier"
```

---

# Stage 3 — Crossing elimination

## Task 3.1: Crossing term in the best-of-passes selector (Part A)

Add the arrangement's crossing count as lex **level 2** in `global_key` only (between
`hard_overlaps` and `soft`). This biases best-of-passes snapshot selection toward
crossing-free layouts going into legalize. It is NOT added per-slot (would be O(n³·8)).

**Files:**
- Modify: `src/side_select.jl` (`global_key` returns a 3-tuple)
- Test: `test/test_side_select.jl`

- [ ] **Step 1: Write the failing test**

```julia
@testset "side_select global selection prefers crossing-free arrangements" begin
    # Two labels whose seeds would cross; an equal-overlap (zero) arrangement exists with
    # no crossing. The global key's crossing level must select the crossing-free one.
    anchors = [Point2f(100, 100), Point2f(200, 100)]
    sizes   = [Vec2f(30, 14), Vec2f(30, 14)]
    p       = RepelParams(box_padding = 2.0, point_padding = 2.0, min_segment_length = 2.0)
    ps      = [sizes[i] .+ 2 * Float32(p.box_padding) for i in 1:2]
    bounds  = Rect2f(0, 0, 300, 300)
    # seeds deliberately cross: label 1 seeded right, label 2 seeded left
    seed    = [Vec2f(60, 0), Vec2f(-60, 0)]
    sel = side_select(anchors, sizes, ps, bounds, seed, p)
    conns = [MakieTextRepel.connector_for(anchors[i], sel[i], sizes[i], false, p, p.min_segment_length)
             for i in 1:2]
    @test isempty(MakieTextRepel.find_crossings(conns))
end
```

- [ ] **Step 2: Run to verify it fails**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-objdrive.log; grep -nA3 "crossing-free arrangements" test/output/test-objdrive.log`
Expected: FAIL — without the crossing level, the selector keeps the crossing seed layout.

- [ ] **Step 3: Implement the crossing level**

In `src/side_select.jl`, extend `global_key` to a 3-tuple `(hard, crossings, soft)`. After
the `hard`/`soft` accumulation, build connectors and count crossings:

```julia
    function global_key(s)
        hard = 0
        soft = 0.0
        for i in 1:n
            b   = box_at(anchors[i], s[i], psizes[i])
            bm  = box_at(anchors[i], s[i], sizes[i])
            for j in (i+1):n
                (overlap_push(b, box_at(anchors[j], s[j], psizes[j])) != Vec2f(0, 0)) && (hard += 1)
            end
            for ob in obstacles
                (overlap_push(b, ob) != Vec2f(0, 0)) && (hard += 1)
            end
            for j in 1:n
                j == i && continue
                point_covered(anchors[j], bm, p) && (hard += 1)
            end
            soft += sqrt(Float64(s[i][1])^2 + Float64(s[i][2])^2) + W_side * sel_rank[i]
        end
        conns = [connector_for(anchors[i], s[i], sizes[i], false, params, params.min_segment_length)
                 for i in 1:n]
        crossings = length(find_crossings(conns))
        return (hard, crossings, soft)
    end
```

Julia compares the 3-tuple lexicographically, so `best_key`/`gk` comparisons are unchanged.
`connector_for` and `find_crossings` are in-module; `params` and `params.min_segment_length`
are in scope.

- [ ] **Step 4: Run to verify it passes**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-objdrive.log; grep -E "Test Summary|Fail|Error" test/output/test-objdrive.log`
Expected: PASS, all prior tests intact.

- [ ] **Step 5: Commit**

```bash
git add src/side_select.jl test/test_side_select.jl
git commit -m "feat(side_select): crossing count as lex level 2 in best-of-passes selector"
```

---

## Task 3.2: Swap-to-fixpoint local search (Part B — the crossing-killer)

Extract the legalize/drop loop into a reusable helper, then run an interleaved
swap+legalize local search after it: while crossings remain, try swapping each crossing
pair's offsets, re-legalize, and accept the swap iff the post-legalize `label_cost` Q
strictly improves lexicographically `(overlaps+point_overlaps, crossings, mean_leader)`.
Monotone over a well-ordered key ⇒ terminating and deterministic. Capped; `@warn` on
residual.

**Files:**
- Modify: `src/solvers/projection.jl` (extract helper; insert local search after the loop, before the warn at `:140`)
- Test: `test/test_projection_solver.jl`

- [ ] **Step 1: Write the failing tests**

```julia
@testset "ProjectionSolver: post-legalize swap search reaches zero crossings" begin
    using MakieTextRepel: ProjectionSolver, solve_cluster, label_cost, RepelParams, connector_for, find_crossings
    # Layout where the greedy seed crosses but a swap untangles, with room to stay overlap-free.
    bounds  = Rect2f(0, 0, 400, 200)
    anchors = [Point2f(120, 100), Point2f(280, 100)]
    sizes   = [Vec2f(50, 18), Vec2f(50, 18)]
    params  = RepelParams(box_padding = 4.0, point_padding = 2.0, min_segment_length = 4.0)
    offs, dropped, _, _ = solve_cluster(ProjectionSolver(params), anchors, sizes, bounds)
    q = label_cost(anchors, sizes; offsets = offs, bounds = bounds, dropped = dropped,
                   box_padding = params.box_padding, point_padding = params.point_padding,
                   min_segment_length = params.min_segment_length)
    @test q.crossings == 0
    @test q.overlaps == 0          # zero-overlap guarantee survives the swap search
end

@testset "ProjectionSolver: swap search is deterministic" begin
    using MakieTextRepel: ProjectionSolver, solve_cluster, RepelParams
    bounds  = Rect2f(0, 0, 400, 400)
    anchors = [Point2f(100 + 7i, 200 + 11*(-1)^i) for i in 1:8]
    sizes   = [Vec2f(44, 16) for _ in 1:8]
    params  = RepelParams(box_padding = 4.0, point_padding = 2.0, min_segment_length = 4.0)
    a = solve_cluster(ProjectionSolver(params), anchors, sizes, bounds).offsets
    b = solve_cluster(ProjectionSolver(params), anchors, sizes, bounds).offsets
    @test a == b
end
```

- [ ] **Step 2: Run to verify they fail (or that crossings remain)**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-objdrive.log; grep -nA3 "swap search" test/output/test-objdrive.log`
Expected: the zero-crossings test FAILs (residual crossing) under the pre-Part-B pipeline.
(If the fixture happens to be crossing-free already, adjust anchor spacing so the greedy
seed produces a crossing before implementing — verify by printing `q.crossings` first.)

- [ ] **Step 3: Extract `legalize_and_drop` and add the local search**

In `src/solvers/projection.jl`, refactor the body of `solve_cluster`. Replace the legalize/
drop `while true` loop (lines 105-138) with a call to a new nested helper that returns the
result, then run the swap search. First add the helper as a `let`-free nested function
inside `solve_cluster` (after the `offsets` are first computed, before the existing loop):

```julia
    # Run the legalize → over-capacity-drop loop to convergence on a *given* offsets vector.
    # Returns (offsets, dropped, lz). Pure w.r.t. its inputs (mutates only locals).
    function legalize_and_drop(start_offsets::Vector{Vec2f})
        offs = copy(start_offsets)
        drp  = falses(n)
        local lz = (; offsets = offs, residual = 0f0, rounds_used = 0)
        while true
            act = Int[i for i in 1:n if !drp[i]]
            m = length(act); k = length(obstacles)
            w_anchors = Vector{Point2f}(undef, m + k)
            w_offsets = Vector{Vec2f}(undef, m + k)
            w_psizes  = Vector{Vec2f}(undef, m + k)
            w_fixed   = falses(m + k)
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
            lz = legalize(w_anchors, w_offsets, w_psizes, bounds;
                          fixed = w_fixed, only_move = p.only_move)
            for (t, i) in enumerate(act)
                offs[i] = lz.offsets[t]
            end
            (lz.residual ≤ 0.5f0 || count(!, drp) ≤ 1) && break
            idx = drop_most_overlapped!(drp, anchors, offs, psizes, pin_mask, obstacles)
            idx == 0 && break
        end
        return (offs, drp, lz)
    end
```

Then replace the old inline loop with:

```julia
    offsets, dropped, lz = legalize_and_drop(offsets)

    # Stage 3 Part B: swap-based local search to drive crossings to zero. Accept a swap iff
    # the post-legalize Q strictly improves lexicographically (overlaps dominate crossings
    # dominate leader), so the search is monotone over a well-ordered key — terminating and
    # deterministic. Capped by UNCROSS_ROUNDS; residual crossings are an honest escape hatch
    # (crossing-free and overlap-free can genuinely conflict in bounds-tight scenes).
    UNCROSS_ROUNDS = 50
    qkey(offs, drp) = let q = label_cost(anchors, sizes; offsets = offs, bounds = bounds,
                                         dropped = drp, box_padding = p.box_padding,
                                         point_padding = p.point_padding,
                                         min_segment_length = p.min_segment_length)
        (q.overlaps + q.point_overlaps, q.crossings, q.mean_leader)
    end
    for _ in 1:UNCROSS_ROUNDS
        conns = [connector_for(anchors[i], offsets[i], sizes[i], dropped[i], p, p.min_segment_length)
                 for i in 1:n]
        X = find_crossings(conns)
        isempty(X) && break
        curkey = qkey(offsets, dropped)
        improved = false
        for (i, j) in X
            (pin_mask !== nothing && (pin_mask[i] || pin_mask[j])) && continue
            trial = copy(offsets)
            swap_positions!(trial, anchors, i, j)
            toffs, tdrp, tlz = legalize_and_drop(trial)
            if qkey(toffs, tdrp) < curkey
                offsets, dropped, lz = toffs, tdrp, tlz
                improved = true
                break
            end
        end
        improved || break
    end

    if lz.residual > 0.5f0
        @warn "ProjectionSolver: residual overlap after dropping; scene over-capacity for bounds=$bounds"
    end
```

(The existing `@warn` and the `q = label_cost(...)` / `s.stats[] = ...` writeback at lines
140-148 stay; they now run on the post-swap-search `offsets`/`dropped`/`lz`, so `solve_stats`
reflects the final layout.)

- [ ] **Step 4: Run to verify they pass**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-objdrive.log; grep -E "Test Summary|Fail|Error" test/output/test-objdrive.log`
Expected: PASS — zero crossings, zero overlaps, deterministic, and the existing
ProjectionSolver tests (warm-start, pinned, over-capacity, all-pinned, single-label) intact.
The warm-start test legalizes only (init_state path) — confirm the swap search still runs on
that path without re-siding pinned labels (pinned pairs are skipped in the swap loop).

- [ ] **Step 5: Commit**

```bash
git add src/solvers/projection.jl test/test_projection_solver.jl
git commit -m "feat(projection): swap-to-fixpoint local search drives crossings to zero"
```

---

## Task 3.3: Escape-hatch test + regenerate the hero image

**Files:**
- Test: `test/test_projection_solver.jl`
- Regenerate: `assets/example.png` via `examples/readme_example.jl`

- [ ] **Step 1: Write the escape-hatch test**

```julia
@testset "ProjectionSolver: over-capacity scene stops at a fixpoint, stays overlap-free" begin
    using MakieTextRepel: ProjectionSolver, solve_cluster, label_cost, RepelParams
    bounds  = Rect2f(0, 0, 80, 80)                 # tiny bounds, 8 coincident anchors
    anchors = [Point2f(40, 40) for _ in 1:8]
    sizes   = [Vec2f(30, 14) for _ in 1:8]
    params  = RepelParams(box_padding = 4.0, point_padding = 2.0, min_segment_length = 4.0)
    res = solve_cluster(ProjectionSolver(params), anchors, sizes, bounds)
    q = label_cost(anchors, sizes; offsets = res.offsets, bounds = bounds, dropped = res.dropped,
                   box_padding = params.box_padding, point_padding = params.point_padding,
                   min_segment_length = params.min_segment_length)
    @test q.overlaps == 0                          # survivors overlap-free (zero-overlap wins conflicts)
    @test any(res.dropped)                         # over-capacity ⇒ at least one drop, no hang
end
```

- [ ] **Step 2: Run to verify it passes**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-objdrive.log; grep -nA3 "over-capacity scene stops" test/output/test-objdrive.log`
Expected: PASS — the search terminates (no infinite loop) and survivors are overlap-free.

- [ ] **Step 3: Regenerate the hero image**

Run: `julia --project=. examples/readme_example.jl`
Expected: prints `wrote .../assets/example.png`. Open the PNG; visually confirm the middle
(`textrepel!`) and right (`annotation!`) panels show labels clear of markers and no leader
crossings.

- [ ] **Step 4: Final full-suite run**

Run: `julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee test/output/test-objdrive.log; grep -E "Test Summary|Fail|Error" test/output/test-objdrive.log`
Expected: whole suite PASS, zero failures/errors.

- [ ] **Step 5: Commit**

```bash
git add test/test_projection_solver.jl assets/example.png
git commit -m "test(projection): over-capacity escape hatch; regenerate hero image"
```

---

## Self-review notes (for the implementer)

- **Determinism**: every new term is integer counts or a fixed-`W_side` scalar folded into
  existing `Float64`/tuple arithmetic; all iteration is index-ordered; the swap search
  accepts the first improving swap in `find_crossings`'s lex order. No RNG anywhere. The
  `bounds === nothing` force-solver path is untouched.
- **Zero-overlap guarantee**: the swap search only ever adopts a `legalize_and_drop` result,
  and rejects any swap that raises `overlaps` (lex level 1). The final layout is always a
  legalized one.
- **If `point_overlaps > 0` survives Task 1.5**: do not weaken the test. That is the
  legalize-erasure case the spec flags; escalate to point-aware legalize (foreign anchors as
  fixed pseudo-nodes with own-anchor exclusion) as a follow-up before claiming Stage 1 done.
- **Cost**: the swap search calls `label_cost` per trial swap (O(crossings·n²) per round,
  capped at `UNCROSS_ROUNDS`). Fine at fixture/hero scale; revisit only if `n` grows large.
