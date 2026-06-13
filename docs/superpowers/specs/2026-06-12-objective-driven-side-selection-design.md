# Objective-Driven Side Selection — Design

*2026-06-12. Enrich the ProjectionSolver's discrete placement objective so the engine
optimizes the quality it already measures: marker-avoidance, side-readability, and
crossings folded into one cost, staged for incremental landing.*

## Where we are (and why this)

The v0.3 `ProjectionSolver` (`side-select → repair → legalize → drop`) gives a
deterministic, zero-overlap placement. But three gaps limit the *quality* of that
placement, all traceable to one root cause: **the cost functional `Q` we measure is
disconnected from the objective the engine steers by.**

- `cost.jl`'s `Q` reports `(overlaps, mean_leader, crossings)` — read-only diagnostics.
- `side_select` actually minimizes a *different, simpler* objective:
  `label-label overlap·W + ‖offset‖` (`src/side_select.jl`).
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
objective into a single unified cost whose terms `cost.jl` reports separately. This is
**Approach A** (enrich the discrete objective, keep the deterministic greedy
best-of-passes search). No continuous solver, no annealing — those (Approach B/C) are
out of scope and only justified if A leaves visible failures.

## The unified objective

`side_select` minimizes a single weighted scalar per slot assignment:

```
J(assignment) = W_lap · (label–label overlap pairs)
              + W_pt  · (label–marker point overlaps)     ← NEW (Stage 1)
              + W_x   · (leader crossings)                ← NEW (Stage 3)
              + W_side· (Imhof rank of chosen slot)       ← NEW (Stage 2)
              + ‖offset‖                                  (finest tiebreak, existing)
```

**Weight ordering is the contract:** `W_lap ≈ W_pt ≫ W_x ≫ W_side`, with `‖offset‖` as
the leader-length tiebreak below all of them. A readability bias must **never** cause a
real overlap. Starting values (tuned empirically against `Q` on the hero example):

| Weight  | Start | Rationale |
|---------|-------|-----------|
| `W_lap` | 1000  | Existing `overlap_weight`. |
| `W_pt`  | 1000  | **Equal to `W_lap`** (decided): a label covering a data point is as misleading as covering another label. |
| `W_x`   | 200   | Crossings are aesthetic, below hard overlaps but above readability. |
| `W_side`| 1.5   | px-equivalent per rank step (TR=0 … TL=7); only tips otherwise-near-equal slots. |

`cost.jl` stays **multi-component and read-only** — it reports the *raw counts* of the
same phenomena the engine weights (`overlaps`, `point_overlaps`, `mean_leader`,
`crossings`), never collapsing them to a scalar. Weights live in the engine only. This
is how "measure = optimize" without violating the never-collapse-`Q` principle.

### Where the inputs come from (no new API for Stages 1 & 3)

- **Markers = anchors.** `side_select` already receives the full `anchors::Vector{Point2f}`.
  The set of markers a label `i` must avoid is `{anchors[j] : j ≠ i}` — a label never
  avoids its *own* anchor (the leader attaches there).
- **Marker keep-out = `point_padding`** (decided, default 2px). `psizes = sizes + 2·point_padding`
  already exists in `solve_cluster` (`src/solvers/projection.jl:83`); we treat each
  foreign anchor as a zero-size point inflated by `point_padding`. The recipe does not
  know the scatter's `markersize`, and reusing `point_padding` avoids new surface area.
- **Crossings** are computed from the candidate arrangement via the existing
  `connector_for` + `find_crossings` (same path `cost.jl` already uses).

## Stage 1 — marker / point avoidance *(biggest visible win, ship first)*

**`src/side_select.jl`:** add a point-overlap term to both the per-slot greedy cost and
`global_cost`. For candidate box `b = box_at(anchors[i], o, psizes[i])`, count foreign
anchors inside it:

```julia
for j in 1:n
    j == i && continue
    point_in_box(anchors[j], b) && (ov += 1)   # weighted by W_pt = overlap_weight
end
```

`point_in_box` is a trivial AABB containment test (anchor is a point; `psizes` already
carries the `point_padding` inflation, so the keep-out radius is baked into `b`).

**`legalize` stays point-unaware** for now. Its displacements are small (it only clears
residual *label-label* penetration), so re-covering a marker is rare. Making legalize
point-aware is an honest deferral — revisit only if Stage 1's `Q` shows markers
re-covered after legalize.

**`src/cost.jl`:** add a `point_overlaps::Int` component (count of label-box ∩
foreign-anchor, same `point_padding` inflation) to the returned NamedTuple. Update the
`solve_stats` shape (`(; iter, residual, overlaps, point_overlaps, mean_leader, crossings, dropped)`)
in `src/solvers/projection.jl` and `src/annotation_algorithm.jl`, and the stability
canary in `test/test_annotation_algorithm.jl`.

**Tests (`test/test_side_select.jl`, `test/test_cost.jl`):** a label seeded onto a
foreign marker re-sides off it; the label's own anchor is *not* avoided; determinism
preserved; `Q.point_overlaps` drops to ~0 on a fixture with markers in label paths.

## Stage 2 — side readability

**`src/side_select.jl`:** add `W_side · rank(slot)` to the per-slot and global cost,
where `rank` is the index of the slot's direction in `IMHOF_ORDER`
(`TR > R > T > BR > L > BL > B > TL`, so TR=0 … TL=7). Gentle by construction: with
`W_side ≈ 1.5` px/step it only tips slots that are otherwise within a couple px of each
other on leader length, and is dominated by any `W_lap`/`W_pt` overlap term.

**Tests:** among equal-overlap, equal-leader candidate slots, TR/T is chosen over B/BL;
a real overlap-avoidance move is never overridden by the side preference (assert a
forced-overlap fixture still re-sides despite the readability pull).

## Stage 3 — crossing penalty + post-legalize re-check

**Discrete penalty (`src/side_select.jl`):** fold the arrangement's crossing count into
`global_cost` (via `connector_for` + `find_crossings`), so the best-of-passes search
prefers crossing-free arrangements from the start. The existing mid-pipeline
`repair_crossings!` is retained.

**Post-legalize re-check (`src/solvers/projection.jl`):** decided to include now, hard-capped.
After `legalize`, recount crossings on the legalized offsets. If legalize reintroduced
any:

1. Run **one** bounded `repair_crossings!` 2-opt swap pass on the legalized layout.
2. Re-run `legalize` **once** to re-clear any overlap the swaps created.
3. Accept the residual; `@warn` if crossings remain.

The hard cap of **one** re-check cycle prevents ping-pong (swap → legalize → new
crossing → swap …). This is best-effort "kill crossings for real" with a deterministic
backstop, consistent with `repair_crossings!`'s existing cap-and-warn philosophy.

**Tests (`test/test_projection_solver.jl`):** an arrangement with an avoidable crossing
resolves crossing-free; a fixture where legalize reintroduces a crossing triggers exactly
one re-check; the cap is respected (no infinite loop); determinism preserved.

## Measurement & guardrails

- **Scoreboard:** extend `examples/readme_example.jl` (or a sibling scratch script) to
  print the extended `Q` — `overlaps / point_overlaps / mean_leader / crossings` — for
  the hero dataset before and after each stage. **Never commit a stage that regresses an
  earlier term.** This is the acceptance gate per stage.
- **Determinism contract preserved.** All new terms are additive in the *discrete*
  phase; `legalize` itself is untouched except the capped Stage-3 re-check, which only
  runs when a crossing is detected post-legalize. The `bounds === nothing` force-solver
  path is entirely unaffected.
- **Zero-overlap guarantee preserved.** `legalize` remains the final separation pass;
  the Stage-3 re-check ends with a `legalize` call, so the output is still legalized.
- Regenerate `assets/example.png` at the end (`julia --project=. examples/readme_example.jl`).

## Non-goals (out of scope)

- **Approach B (stronger combinatorial search — annealing/multi-restart).** Greedy
  best-of-passes stays. Only escalate if A leaves specific clusters visibly bad.
- **Approach C (continuous objective-driven solver).** Reintroduces the jitter /
  non-determinism we deliberately left behind. Out.
- **Point-aware `legalize`.** Deferred; revisit only if Stage 1 shows post-legalize
  marker re-covering.
- **User-facing weight attributes / label priority.** Weights are internal constants
  with the documented starting values. A public tuning API is a separate future feature.
- **Real scan-line VPSC.** Still deferred (unchanged from v0.3).
- **`markersize`-aware keep-out.** Reuse `point_padding`; the recipe doesn't know the
  scatter's markersize.

## Build sequence

Three independently shippable, independently testable stages, each gated on its `Q`
scoreboard not regressing earlier terms:

1. **Stage 1** — marker avoidance (`side_select` point term + `cost.jl` `point_overlaps`
   + `solve_stats` shape + tests). Biggest visible win.
2. **Stage 2** — side readability (`W_side` rank term + tests).
3. **Stage 3** — crossing penalty in `global_cost` + capped post-legalize re-check + tests.
4. Regenerate `assets/example.png`; final full-suite run.
