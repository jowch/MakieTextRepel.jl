# Objective-Driven Side Selection — Design

*2026-06-12. Enrich the ProjectionSolver's discrete placement objective so the engine
optimizes the quality it already measures: marker-avoidance, side-readability, and
crossings folded into one objective, staged for incremental landing.*

*Rev. 2 (2026-06-12): revised after a three-reviewer validation pass. Fixes a blocker
(marker keep-out source), replaces the fragile weighted-scalar objective with a hybrid
lexicographic one, scopes the crossing term to the global phase only, enumerates all
`solve_stats` sites, and strengthens the acceptance gate to a multi-fixture battery.*

*Rev. 3 (2026-06-12): Stage 3 escalated from a single capped best-effort re-check to a
swap-based local search that iterates to a joint (overlap-free ∧ crossing-free) fixpoint —
a scoped piece of Approach B.*

*Rev. 4 (2026-06-12): after a second three-reviewer pass. Objective is now **fully
lexicographic with no weights** (the `W_side` px-weight was nearly inert and is replaced by a
`rank` lex level that never lengthens a leader). Stage 3 Part B's pseudocode is corrected to
scan all crossing pairs (not just the first), its termination argument is re-grounded on
finiteness of the swap-reachable layout set (not a false "well-ordered float domain"), its
accept-key gains a top-level drop-count guard (never drop a label to kill a crossing) and is
reconciled to the rank-free `label_cost` Q, and the crossing-free promise is scoped to
**swap-reachable** inputs (2-opt cannot untangle a 3-cycle; 3-opt is a tracked future option).*

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
objective so it accounts for the same phenomena `cost.jl` reports (**Approach A** — enrich
the discrete objective, keep the deterministic greedy best-of-passes search), **plus a
scoped piece of Approach B for crossings**: a swap-based local search that iterates to a
joint overlap-free ∧ crossing-free fixpoint (Stage 3). The local search is confined to the
offset-swap neighborhood and driven by the same lexicographic objective; full Approach B
(annealing / multi-restart over slot assignments) and Approach C (continuous solver) remain
out of scope.

## The unified objective — fully lexicographic, no weights

The original draft used a single weighted scalar `W_lap·lap + W_pt·pt + W_x·x + W_side·rank + ‖offset‖`.
Review showed this is fragile: aggregated crossings could outweigh a real overlap, and the
`W_side` px-weight was simultaneously "above and below" leader length in the prose. A later
review round then showed `W_side` as a small additive px term is **nearly inert** — for any
wider-than-tall label the geometric leader gap between a corner slot (TR) and an axis slot
(T) dwarfs `7·W_side`, so it can only ever break near-equal-leader ties. So we drop weights
entirely and make the objective a **pure lexicographic tuple**:

```
key(assignment) = ( hard_overlaps,   ← level 1 (integer; strict dominance)
                    crossings,        ← level 2 (integer; GLOBAL phase only)
                    leader,           ← level 3 (Float; total/own leader length)
                    rank )            ← level 4 (integer; Imhof readability, pure tiebreak)

where  hard_overlaps = label–label overlap pairs + label–marker point overlaps
       rank          = Σ IMHOF rank index of each label's slot (TR=0 … TL=7)
```

Compared with Julia's native tuple `<` (lexicographic). Properties:

- **Overlap is never traded for anything.** `hard_overlaps` is the integer compared first;
  no aggregate of lower levels can override it. The "never cause a real overlap" contract is
  *provable*, not weight-dependent.
- **`W_pt = W_lap` (decided) is exact** — label and marker overlaps sum into the *same*
  level-1 integer; covering a data point is as bad as covering a label.
- **Crossings sit strictly below overlaps** and are evaluated only in the global/best-of-passes
  phase, so the O(n³·8) per-slot trap is avoided.
- **Readability never lengthens a leader.** `rank` is level 4, strictly below `leader`, so it
  only decides among *exactly* equal-leader slots (e.g. T vs B, R vs L, which have identical
  leader magnitude) — picking the upper/right one. This formalizes and tests the readability
  preference that the existing `IMHOF_ORDER` iteration order already produced implicitly, and
  additionally fixes it at the global best-of-passes level (where two equal-leader arrangements
  differing only in side would otherwise tie and keep whichever was seen first). Its visual
  effect is deliberately subtle; it costs zero leader length.
- **Stages stay independent for the hard levels.** Adding a lower level (crossings in Stage 3,
  rank in Stage 2) provably cannot change a higher level's outcome, so Stage 1's overlap
  results are never retroactively invalidated. (Lower levels *can* shift placements among
  ties — acknowledged in the build sequence.)

There are **no tunable weights anywhere** in the side-select objective — every level is an
integer count or raw leader length.

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

A single predicate backs both the engine term and the `cost.jl` count, so the **new marker
term** agrees between objective and scoreboard:

```julia
point_covered(p::Point2f, box::Rect2f, padding::Real) :: Bool   # added to src/geometry.jl
```

True iff `p` lies strictly inside `box` expanded by `padding`. Both `side_select` (Stage 1)
and `label_cost` (Stage 1) call this exact function with the unpadded text box
`box_at(anchor, o, sizes[i])` and `padding = point_padding`. No `point_in_box` helper exists
today (`src/geometry.jl` has only `clip_to_box_edge`'s inline inside-test); it is added once
and shared.

*Threshold note:* this predicate uses strict containment (penetration > 0), matching
`side_select`'s existing label–label term (`overlap_push(...) != 0`, also > 0). It does **not**
match `cost.jl`'s label–label `overlaps`, which ignores sub-`0.5px` touches (`cost.jl:41`).
That `>0px` vs `>0.5px` divergence on the *label–label* term is pre-existing and out of scope;
sharing `point_covered` only guarantees the *marker* term agrees between engine and scoreboard,
which is what matters for "measure = optimize" on the new term.

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

New shape (7 fields). The `ProjectionStats` typed alias is positional, so fix one canonical
field order and use it at every site — insert `point_overlaps` right after `overlaps`:
`(:overlaps, :point_overlaps, :mean_leader, :crossings, :iter, :residual, :dropped)`. The
keyword-constructed literals (`(; overlaps=…, point_overlaps=…, …)`) are order-insensitive,
but the typed alias and the all-pinned exact-`==` test are not — match the canonical order.

**Tests:**
- `test/test_side_select.jl` — a label seeded onto a foreign marker re-sides off it; the
  label's *own* anchor is **not** avoided; determinism preserved.
- `test/test_cost.jl` — `point_overlaps` counts the shared predicate correctly; agrees with
  `side_select`'s count on a shared fixture.
- `test/test_annotation_algorithm.jl:52-53` — update the stability-canary field set; and
  `:118-119` — update the all-pinned **exact-tuple-equality** assertion to the new shape
  (both are hard failures otherwise).

## Stage 2 — side readability (pure lexicographic tiebreak)

**`src/side_select.jl`:** add `rank` as the **lowest lexicographic level** (below `leader`),
where `rank` is the slot's index in `IMHOF_ORDER` (`src/init.jl:4`; TR=0 … TL=7). No weight.
Because it sits strictly below `leader`, it decides only among slots with *exactly equal*
leader length — which is precisely the readable symmetric pairs (T vs B, R vs L, both have
identical leader magnitude) — picking the upper/right one. It can never lengthen a leader.

*Honest scope:* the existing candidate iteration already runs in `IMHOF_ORDER` with a strict
`<`, so per-slot exact ties were already broken readably. The real value of making `rank` an
explicit level is (a) it is now documented and tested rather than an accidental side-effect,
and (b) it fixes the **global best-of-passes** selection, where two whole arrangements with
equal `(hard_overlaps, crossings, leader)` but different total rank would otherwise tie and
keep whichever pass was seen first (which can be the less-readable seed). Visual effect is
subtle by design.

**Tests:** for a label whose T and B slots both fit and have equal leader, the upper slot (T,
lower rank) is chosen; readability never overrides overlap-avoidance (a forced-overlap fixture
still re-sides — level 1 dominates level 4); the rank level never lengthens a leader (a slot
with a strictly shorter leader is always chosen over a more-readable longer one).

## Stage 3 — crossing elimination via swap-based local search

Two parts: a cheap global-phase term that *prefers* crossing-free arrangements going into
legalize, and a post-legalize **local search** that actively drives crossings to zero.

**Part A — discrete penalty (`src/side_select.jl`):** add the arrangement's crossing count
as lex **level 2** in `global_cost` **only** (the best-of-passes selector), via `connector_for`
+ `find_crossings`. It is **not** added to the per-slot greedy loop — doing so would rebuild
the global connector set inside `passes · n · 8` slot trials (O(n³·8)) to compute a property
that only ranks whole-arrangement snapshots. So its influence is real but bounded: it
re-ranks the ≤`passes` snapshots, not individual greedy moves. (`connector_for`/`find_crossings`
are in-module; `side_select` already has `anchors`, `sizes`, `params` in scope.) This gives
the local search (Part B) a good starting point but is not where crossings are eliminated.

**Part B — post-legalize swap-to-fixpoint (`src/solvers/projection.jl`):** the real
crossing-killer. After the legalize/drop loop produces an overlap-free layout, run an
**interleaved swap+legalize local search** to a joint fixpoint:

```
repeat up to UNCROSS_ROUNDS (cap, e.g. 50):
    X = find_crossings(current legalized offsets)
    X is empty  ⇒  break (success: crossing-free ∧ overlap-free)
    improved = false
    for each crossing pair (i, j) in X, in index-sorted order:
        candidate = swap offsets[i] ↔ offsets[j], then re-run the legalize/drop loop
        if swapkey(candidate) < swapkey(current):       # strict lexicographic improvement
            adopt candidate; improved = true; break      # restart the outer scan
    improved == false  ⇒  break (local fixpoint)
```

The inner loop scans **all** current crossing pairs for the *first* improving swap and
restarts on success; only a full scan with no improving swap breaks. (An earlier draft picked
only the first pair, which could spin on a non-improving first pair while a later pair would
have helped — fixed here.)

**`swapkey` (post-legalize, rank-free).** Reuses the read-only `label_cost` Q, with a
drop-count guard on top:

```
swapkey(layout) = ( dropped_count,                  ← never drop a label to fix a crossing
                    overlaps + point_overlaps,      ← hard overlaps (lex level 1 of the engine)
                    crossings,
                    mean_leader )                   ← rank-free: cost.jl is rank-unaware by design
```

`dropped_count` is the top level so a swap that reduces crossings by *dropping a label* can
never be accepted (it strictly raises `dropped_count`). The soft tail is `mean_leader` only —
`cost.jl` does not carry Imhof rank, and the Stage-2 rank tiebreak is a side-selection concern,
not a swap-search one; reconciling the two keeps the swap search reusing the existing Q
verbatim. (This intentionally differs from the side-select key, which has the rank level.)

**Why this terminates and is deterministic.** "Swap `offsets[i]↔[j]` then run the
deterministic legalize/drop loop" is a deterministic function of the pre-swap offset vector,
and swaps only permute a finite set of offset assignments, so the set of layouts reachable by
swap sequences is **finite**. Each adopted swap strictly decreases `swapkey` (a tuple over
that finite set), and a strictly-decreasing walk over a finite ordered set cannot revisit a
layout — so it terminates, at crossing-free or a local fixpoint, within `UNCROSS_ROUNDS`.
(Termination rests on **finiteness of the swap-reachable layout set**, not on well-ordering of
the float `mean_leader` — a strictly-decreasing real sequence alone need not be finite.) Note
legalize re-projects the *whole* active set, so an adopted swap can move non-swapped labels
too; `swapkey` is therefore a functional of the entire legalized layout, which is exactly what
the finite-reachable-set argument needs. Pair/scan order is index-sorted, no RNG — determinism
preserved.

**The honest caveat (escape hatch).** Two ways the search can stop with residual crossings,
both reported via `@warn` + `solve_stats.crossings`:
1. **Conflict.** Removing a crossing forces an overlap that legalize can only clear by dropping
   a label (over-capacity, bounds-tight). The drop-count guard / overlap level reject such a
   swap, so overlap-freeness and the no-extra-drop guarantee both win.
2. **Neighborhood limit.** 2-opt offset swaps untangle any *pairwise*-untangleable crossing,
   but provably cannot resolve a 3-cycle of mutually-crossing leaders (that needs a 3-way
   rotation). Such a configuration stalls at a swap-local fixpoint even though a crossing-free
   arrangement exists in a larger neighborhood.

So the honest promise is **crossing-free whenever a swap-reachable crossing-free arrangement
exists** — which covers the overwhelming majority of real scatter layouts — not an
unconditional guarantee. Extending the neighborhood to 3-opt rotations is a tracked future
option if 2-opt proves insufficient in practice (it has not on the fixtures). The final layout
is always the last legalize/drop output, so the **zero-overlap guarantee is unaffected**: when
goals conflict, overlap-freeness (and no extra drops) wins.

`solve_stats.crossings` and `.residual` reflect the post-local-search final layout. The
existing `repair_crossings!` mid-pipeline pass (pre-legalize) is retained as a cheap warm
start; Part B subsumes its role as the closer.

**Tests (`test/test_projection_solver.jl`):**
- A fixture with an avoidable crossing converges to **zero** crossings (not merely "fewer").
- A fixture where the greedy seed crosses but a single swap untangles: assert crossing-free
  output and that overlaps stayed at zero.
- Determinism: identical input → identical offsets across runs.
- Termination: the search respects `UNCROSS_ROUNDS` and never loops (assert rounds-used ≤ cap
  on a stress fixture).
- Escape hatch: a deliberately over-capacity / crossing-vs-overlap-conflicting fixture stops
  at a local fixpoint, drops/over-capacity behavior unchanged, and `@warn`s rather than
  hanging or violating zero-overlap.

## Measurement & guardrails

- **Scoreboard is a committed multi-fixture battery, not one eyeballed example.** Reuse the
  fixture set already in `test/test_projection_solver.jl` (the §7e sparse / tight-knot /
  dense-collinear scenes) and add an aggregate non-regression `@testset`: for each new
  stage, assert no per-term regression **summed across fixtures** (e.g.
  `Σ overlaps`, `Σ point_overlaps`, `Σ crossings` non-increasing; `Σ mean_leader` within
  tolerance). Store baseline Q values in the test. The hero PNG stays a human gut-check, not
  the gate. **Measure `point_overlaps` post-legalize** (not just on the discrete output) so
  Stage 1's legalize-erasure risk is actually caught.
- **Determinism contract preserved.** Every objective level is an integer count or raw
  leader length compared via native tuple `<`; iteration stays index-ordered, no RNG. The
  Stage-3 swap search is gated on a deterministic recount and a deterministic legalize, scans
  pairs in index order, and is capped by `UNCROSS_ROUNDS`. The `bounds === nothing` force path
  is untouched.
- **Zero-overlap guarantee preserved** (see Stage 3).
- Regenerate `assets/example.png` at the end (`julia --project=. examples/readme_example.jl`).

## Non-goals (out of scope)

- **Full Approach B (annealing / multi-restart over slot assignments) and Approach C
  (continuous solver).** The greedy best-of-passes side-selection stays. *Exception:* Stage 3
  uses a **scoped** piece of Approach B — a swap-neighborhood local search for crossing
  elimination (above). The broader slot-assignment search space and stochastic methods remain
  out; only escalate further if this leaves visible failures.
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
3. **Stage 3** — crossing term in `global_cost` (Part A, global phase only) + swap-based
   post-legalize local search to a joint overlap-free ∧ crossing-free fixpoint (Part B,
   `UNCROSS_ROUNDS` cap, lex-gated acceptance, escape-hatch `@warn`) + tests.
4. Regenerate `assets/example.png`; final full-suite run.
