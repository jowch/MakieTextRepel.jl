# Objective-Driven Side Selection — Design

*2026-06-12. Enrich the ProjectionSolver's discrete placement objective so the engine
optimizes the quality it already measures: marker-avoidance, side-readability, and
crossings folded into one objective, staged for incremental landing.*

*Rev. 2 (2026-06-12): revised after a three-reviewer validation pass. Fixes a blocker
(marker keep-out source), replaces the fragile weighted-scalar objective with a hybrid
lexicographic one, scopes the crossing term to the global phase only, enumerates all
`solve_stats` sites, and strengthens the acceptance gate to a multi-fixture battery.*

## Where we are (and why this)

The v0.3 `ProjectionSolver` (`side-select → repair → legalize → drop`) gives a
deterministic, zero-overlap placement. But three gaps limit the *quality* of that
placement, all traceable to one root cause: **the cost functional `Q` we measure is
disconnected from the objective the engine steers by.**

- `cost.jl`'s `Q` reports `(overlaps, mean_leader, crossings)` — read-only diagnostics.
- `side_select` actually minimizes a *different, simpler* objective:
  `label-label overlap·W + ‖offset‖` (`src/side_select.jl:100-107`).
- `legalize` minimizes displacement-from-current (Dykstra), not `Q`.

Concretely this produces three visible defects on the clustered-knots hero example
(`examples/readme_example.jl`):

1. **Labels avoid other labels but not scatter markers.** `side_select` scores
   label-box vs. label-box and explicit `obstacles` only (`src/side_select.jl:100-106`).
   The old force solver had label↔point repulsion over *all* anchors; the
   ProjectionSolver dropped it. A label can sit squarely on another node's marker.
2. **Crossings are only "kept no worse," not eliminated.** `legalize` runs after
   `repair_crossings!` and can reintroduce crossings (see CLAUDE.md). Final leaders
   can still cross.
3. **Readability is sacrificed for leader length.** Imhof's "prefer above/top-right"
   is overridden by shortest-leader minimization, so labels drift to whichever side is
   geometrically cheapest even when it reads worse.

## Goal

Reconnect measurement and optimization by enriching the **discrete** `side_select`
objective so it accounts for the same phenomena `cost.jl` reports. This is **Approach A**
(enrich the discrete objective, keep the deterministic greedy best-of-passes search).
No continuous solver, no annealing — those (Approach B/C) are out of scope and only
justified if A leaves visible failures.

## The unified objective — hybrid lexicographic

The original draft used a single weighted scalar `W_lap·lap + W_pt·pt + W_x·x + W_side·rank + ‖offset‖`.
Review showed this is fragile: with `W_x` aggregated over several crossings the scalar
can let crossings outweigh a real overlap, violating the contract that overlap-avoidance
must dominate. We therefore split the objective into a **lexicographic key on the hard
terms** plus a **weighted scalar at the soft bottom**:

```
key(assignment) = ( hard_overlaps,            ← lex level 1 (integer; strict dominance)
                    crossings,                ← lex level 2 (integer; GLOBAL phase only)
                    leader + W_side · rank )   ← lex level 3 (soft scalar; the only weight)

where  hard_overlaps = label–label overlap pairs + label–marker point overlaps
```

Minimized lexicographically: level 1 before level 2 before level 3. Properties this buys
(each addresses a specific review finding):

- **Overlap is never traded for crossings or readability.** `hard_overlaps` is compared
  first as an integer; no aggregate of lower terms can ever override it. The "never cause
  a real overlap" contract is now *provable*, not weight-dependent. (Replaces the brittle
  `W_lap ≫ W_x` magnitude ordering.)
- **`W_pt = W_lap` (decided) is expressed exactly** by summing label and marker overlaps
  into the *same* lex level — covering a data point is as bad as covering a label.
- **Crossings can't outweigh overlaps** (they sit strictly below) and are evaluated only
  in the global/best-of-passes phase (see below), so the O(n³·8) per-slot trap is avoided.
- **Only one tunable weight survives — `W_side`** — and it lives at the soft bottom where
  scale-dependence is acceptable and intended. `W_side ≈ 1.5` px per rank step (ranks 0–7
  from `IMHOF_ORDER`) means the engine will accept up to `7·1.5 = 10.5` px of extra leader
  to reach the most-preferred side. This is a *deliberate* readability-for-leader trade, not
  a pure tiebreak; the prose elsewhere is written to match (no "finest tiebreak" claim).
- **Stages become genuinely independent.** Adding a lower lex level (crossings in Stage 3)
  provably cannot change the level-1 (overlap) outcomes that Stage 1 committed, so prior
  stages' baselines don't get retroactively invalidated by later stages. (The soft level 3
  *can* shift among equal-overlap, equal-crossing slots — acknowledged in the build sequence.)

`cost.jl` stays **multi-component and read-only** — it reports the *raw counts* of the
same phenomena (`overlaps`, `point_overlaps`, `mean_leader`, `crossings`), never
collapsing them. The engine's lexicographic key and `cost.jl`'s separate counts measure
the *same* events via a *shared predicate* (below), which is what makes "measure = optimize"
honest.

### Where the inputs come from

- **Markers = anchors.** `side_select` already receives the full `anchors::Vector{Point2f}`.
  The set of markers a label `i` must avoid is `{anchors[j] : j ≠ i}` — a label never
  avoids its *own* anchor (the leader attaches there).
- **Marker keep-out = `point_padding`** (decided, default 2px). **Correction from rev. 1:**
  the existing `psizes` is inflated by `box_padding` (`src/solvers/projection.jl:82-83`),
  **not** `point_padding`, so the marker test must inflate *explicitly* and cannot reuse
  `psizes`. A foreign anchor `anchors[j]` is "covered" by label `i` at offset `o` iff it
  lies inside `box_at(anchors[i], o, sizes[i])` expanded by `point_padding` on each side —
  mirroring the force solver's point handling (`src/force_model.jl:101,131`). `point_padding`
  is the semantically correct knob (it is the point-clearance radius), and reusing it adds
  no API. `psizes` (box-padding) remains the basis for the label–label term only.
- **Crossings** are computed from the candidate arrangement via the existing
  `connector_for` + `find_crossings` (same path `cost.jl` already uses).

### Shared point-overlap predicate

A single predicate backs both the engine term and the `cost.jl` count, so the objective
and the scoreboard never disagree:

```julia
# anchor covered by a label box, with point_padding keep-out and the same penetration
# threshold the rest of the engine uses (cost.jl ignores sub-0.5px touches).
point_covered(anchor, label_box) :: Bool   # added to src/geometry.jl
```

Both `side_select` (Stage 1) and `label_cost` (Stage 1) call this exact function with the
`point_padding`-expanded label box, using the `>0.5px` inset convention from `cost.jl:41`.
No `point_in_box` helper exists today (`src/geometry.jl` has `clip_to_box_edge`'s inline
inside-test only); it is added once and shared.

## Stage 1 — marker / point avoidance *(biggest visible win, ship first)*

**`src/side_select.jl`:** add the marker term to `hard_overlaps` in both the per-slot
greedy cost (`:100-106`) and `global_cost` (`:80-85`). For candidate box
`b = box_at(anchors[i], o, sizes[i])` expanded by `point_padding`, count foreign anchors
`j ≠ i` with `point_covered(anchors[j], b)`; add that count to the label–label overlap
count at the same lex level. (Note this is O(n) per slot — cheap, unlike the crossing
term — total stays O(passes · n² · 8).)

**`legalize` stays point-unaware in Stage 1** — *consciously, with a measured guard.*
Reviewers flagged that legalize's Dykstra displacements are *not* guaranteed small on dense
knots, so legalize could shove a label back onto a marker and silently erase the discrete
win. We do **not** fold marker-awareness into legalize here because the existing fixed-node
path (`src/solvers/projection.jl:108-124`) treats pseudo-nodes as *global* obstacles, but a
label must avoid only *foreign* anchors, not its own — per-label exclusion is real work, not
free reuse. Instead Stage 1's acceptance gate measures `Q.point_overlaps` **after legalize**
on dense fixtures (below). If that shows material re-covering, the pre-committed follow-up is
point-aware legalize with own-anchor exclusion — tracked explicitly, not as a vague "maybe."

**`src/cost.jl`:** add a `point_overlaps::Int` component to `label_cost`'s returned
NamedTuple, computed with the shared `point_covered` predicate over foreign anchors. Update
the `label_cost` docstring signature (`cost.jl:4-6`).

**`solve_stats` shape change — all sites (reviewer-enumerated):**
1. `src/solvers/projection.jl:9-10` — the `const ProjectionStats = NamedTuple{...}` typed
   alias (field-name tuple **and** the `Tuple{...}` element types).
2. `src/solvers/projection.jl:22-24` — the `ProjectionSolver(params)` constructor's
   zero-init literal (hard construction failure if missed).
3. `src/solvers/projection.jl:147-148` — the real writeback in `solve_cluster`.
4. `src/annotation_algorithm.jl:125-126` — the all-pinned bypass literal (hard failure if
   missed).
5. `src/annotation_algorithm.jl:33-34, 80` — `solve_stats` docstrings (two spots).
6. `src/solvers/projection.jl:13-15` — `ProjectionStats` docstring.

New shape: `(; iter, residual, overlaps, point_overlaps, mean_leader, crossings, dropped)`.

**Tests:**
- `test/test_side_select.jl` — a label seeded onto a foreign marker re-sides off it; the
  label's *own* anchor is **not** avoided; determinism preserved.
- `test/test_cost.jl` — `point_overlaps` counts the shared predicate correctly; agrees with
  `side_select`'s count on a shared fixture.
- `test/test_annotation_algorithm.jl:52-53` — update the stability-canary field set; and
  `:118-119` — update the all-pinned **exact-tuple-equality** assertion to the new shape
  (both are hard failures otherwise).

## Stage 2 — side readability

**`src/side_select.jl`:** add `W_side · rank(slot)` to the soft level-3 scalar
(`leader + W_side·rank`), where `rank` is the slot's index in `IMHOF_ORDER`
(`src/init.jl:4`; TR=0 … TL=7). This is a deliberate trade (up to `7·W_side ≈ 10.5` px of
leader to reach TR), dominated by any level-1 overlap or level-2 crossing by construction.

**Tests:** among slots with equal `hard_overlaps`, the lower-rank (more-readable) slot wins
even at a few px more leader; a forced-overlap fixture still re-sides for overlap-avoidance
despite the readability pull (level 1 dominates level 3).

## Stage 3 — crossing penalty + capped post-legalize re-check

**Discrete penalty (`src/side_select.jl`):** add the arrangement's crossing count as lex
**level 2** in `global_cost` **only** (the best-of-passes selector), via `connector_for` +
`find_crossings`. It is **not** added to the per-slot greedy loop — doing so would rebuild
the global connector set inside `passes · n · 8` slot trials (O(n³·8)) to compute a property
that only ranks whole-arrangement snapshots. So its influence is real but bounded: it
re-ranks the ≤`passes` snapshots, not individual greedy moves. (`connector_for`/`find_crossings`
are in-module; `side_select` already has `anchors`, `sizes`, `params` in scope.)

**Post-legalize re-check (`src/solvers/projection.jl`):** *best-effort, capped — not a
guarantee.* After the legalize/drop loop, recount crossings on the final offsets. If any are
present and look reducible:
1. Run **one** bounded `repair_crossings!` 2-opt swap pass on the legalized layout.
2. Re-run the legalize loop **once** and let its `lz` (residual/rounds) **replace** the prior
   `lz`, so the over-capacity warn (`projection.jl:140`) and `solve_stats` reflect the *final*
   layout.
3. Accept the residual; `@warn` if crossings remain.

The hard cap of **one** cycle bounds work and prevents ping-pong, but does **not** guarantee
zero crossings — `legalize` is crossing-unaware and step 2 can re-slide a swapped label across
a neighbor's leader. The honest framing (matching `repair_crossings!`'s own cap-and-warn) is
"fewer crossings, best-effort," not "kill crossings." The **zero-overlap guarantee survives**
because the cycle ends with a `legalize` call whose residual feeds the same drop/warn path.

**Tests (`test/test_projection_solver.jl`):** an arrangement with an avoidable crossing is
selected crossing-free by the global term; a fixture where legalize reintroduces a crossing
triggers exactly one re-check (cap respected, no loop); `solve_stats.residual` reflects the
post-re-check legalize; determinism preserved.

## Measurement & guardrails

- **Scoreboard is a committed multi-fixture battery, not one eyeballed example.** Reuse the
  fixture set already in `test/test_projection_solver.jl` (the §7e sparse / tight-knot /
  dense-collinear scenes) and add an aggregate non-regression `@testset`: for each new
  stage, assert no per-term regression **summed across fixtures** (e.g.
  `Σ overlaps`, `Σ point_overlaps`, `Σ crossings` non-increasing; `Σ mean_leader` within
  tolerance). Store baseline Q values in the test. The hero PNG stays a human gut-check, not
  the gate. **Measure `point_overlaps` post-legalize** (not just on the discrete output) so
  Stage 1's legalize-erasure risk is actually caught.
- **Determinism contract preserved.** New terms are integer counts and an integer rank
  lookup folded into the existing `Float64` arithmetic class (`side_select.jl:107`);
  iteration stays index-ordered, no RNG. The Stage-3 re-check is gated on a deterministic
  recount and runs a fixed one cycle. The `bounds === nothing` force path is untouched.
- **Zero-overlap guarantee preserved** (see Stage 3).
- Regenerate `assets/example.png` at the end (`julia --project=. examples/readme_example.jl`).

## Non-goals (out of scope)

- **Approach B (annealing/multi-restart) and Approach C (continuous solver).** Greedy
  best-of-passes stays; only escalate if A leaves visible failures.
- **Point-aware `legalize`.** Deferred, but with a concrete trigger (Stage 1's post-legalize
  gate) and a known shape (foreign-anchor pseudo-nodes with own-anchor exclusion — *not* a
  free reuse of the global fixed-node path).
- **User-facing weight attributes / label priority.** `W_side` is an internal constant; the
  hard terms have no weights at all. A public tuning API is a separate future feature.
- **Real scan-line VPSC.** Still deferred (unchanged from v0.3).
- **`markersize`-aware keep-out.** Reuse `point_padding`; the recipe doesn't know the
  scatter's markersize.

## Build sequence

Three stages, each gated on the multi-fixture Q battery. Land-order-independent for the hard
(lexicographic) terms; the soft level-3 scalar may shift placements among equal-overlap,
equal-crossing slots, so re-baseline the *soft* metrics (mean_leader) when Stage 2/3 land —
expected, not a regression.

1. **Stage 1** — marker avoidance: shared `point_covered` predicate (`geometry.jl`),
   `side_select` marker term, `cost.jl` `point_overlaps`, full `solve_stats` shape change
   (6 sites + 3 docstrings), tests, post-legalize gate. Biggest visible win.
2. **Stage 2** — side readability (`W_side` rank term in the soft level, tests).
3. **Stage 3** — crossing term in `global_cost` (global phase only) + capped post-legalize
   re-check + tests.
4. Regenerate `assets/example.png`; final full-suite run.
