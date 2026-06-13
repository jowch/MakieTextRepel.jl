# Marker-clearance floor: point-aware legalize (issue #21)

**Date:** 2026-06-13
**Issue:** #21 — Marker occlusion: enforce a marker-clearance floor (point-aware legalize)
**Status:** Design approved; revised after spec-validation review (0 blockers, 9 major, 18 minor — all addressed below); ready for implementation plan.

## Problem

Scatter markers paint over the start of label text. In the hero image
(`assets/example.png`), `node 11`'s box is covered by `node 8`'s marker (a real
foreign overlap, `point_overlaps = 1`), and labels more generally clip *under*
their own markers. This is **occlusion**, not leader length.

### Root cause (verified against the code)

1. **`legalize` does not treat the anchor as a keep-out.** Its working nodes are
   active label boxes ∪ obstacle pseudo-nodes only (`solvers/projection.jl`,
   `legalize_and_drop`). After `init`/`side_select` place a box at `point_padding`
   from the marker (`slot_offset`, `init.jl`), `legalize` is free to shove it back
   across its own or a foreign anchor — "legalize-erasure."
2. **The swap-uncross loop never re-checks it** — it's gated on `find_crossings`,
   and here `crossings = 0`.
3. **`point_padding` was smaller than a typical marker** — recipe default `2.0`
   vs. Makie default scatter `markersize = 9` (radius 4.5).
4. **`textrepel!` and `scatter!` are separate plots,** so the recipe never sees the
   externally-drawn marker size.

## Solution overview

One mechanism (point-aware legalize) resolves the occlusion; three smaller changes
make it usable and good by default. **Decision summary** (engineering calls made
during review, within the approved direction):

- Marker clearance is a **best-effort floor that never triggers a label drop**
  (unlike label–label overlap, which does). Implemented with a `soft` node class in
  `legalize`.
- The `RepelParams` primitive default for `point_padding` stays `0.0` (it is the
  ForceSolver's repulsion halo); only the two ProjectionSolver **user surfaces** bump
  to `5.0`.
- `markersize`, when set, **overrides** `point_padding` with no conflict warning
  (explicit-vs-default is not reliably detectable in the lift). `margin = 0.5` so
  `markersize = 9 → point_padding = 5.0`, consistent with the bare default.

### 1. Point-aware legalize (the core fix)

In `solvers/projection.jl`'s `legalize_and_drop`, append **every anchor** (all `n`,
own and foreign, regardless of dropped/pinned state — the scatter markers are drawn
for every point) to the legalize working arrays as a **fixed, soft keep-out node**:

- position: the anchor point (offset `(0,0)`),
- half-extent on both axes: `mc = point_padding − box_padding` (signed; may be
  negative),
- `fixed = true` (contributes extents, never moves, not clamped to bounds),
- `soft = true` (see §1a).

**Why `mc = point_padding − box_padding` is exactly right.** `legalize` separates a
pair to a center-to-center gap of `hw_i + hw_j`. A label node carries the *padded*
half-extent `unpadded_half + box_padding`, so a label-vs-marker pair separates to:

```
gap = (unpadded_half + box_padding) + (point_padding − box_padding)
    =  unpadded_half + point_padding
```

i.e. the **unpadded text edge clears the marker center by exactly `point_padding`**,
independent of `box_padding`, for every value including 0. This matches
`point_covered(anchor, unpadded_box, point_padding)` in `geometry.jl` (the predicate
shared by `side_select`'s marker term and `cost.jl`'s `point_overlaps`), so the
geometric floor and the Q scoreboard count the same event. The negative-half-extent
math is sound: `legalize` treats node extents purely numerically, and a negative `mc`
flows correctly through `ox/oy` detection, the `gap` constraint, and the two-fixed-node
skip.

**Own-marker handling.** The geometric floor applies to own and foreign anchors alike
(every anchor gets a keep-out node). The `j == i` skip stays in **scoring only** —
`side_select`'s `ov`/`global_key` `hard` term, `cost.jl`'s `point_overlaps`, and
`drop_most_overlapped!` — so own-marker coverage is never *counted* as a droppable
overlap.

#### 1a. Soft nodes — marker clearance never triggers a drop

**Problem the review surfaced (major):** `legalize`'s returned `residual` is measured
over the full working set, so a movable-label-vs-fixed-marker pair contributes to it.
But `drop_most_overlapped!` scores candidates only over labels + obstacles — it is
blind to marker nodes. If a residual were driven purely by a marker-clearance
violation (e.g. a corner anchor whose clearance is in-bounds-unsatisfiable), the drop
loop would fire, every active label would score `ov = 0`, and the "ties → highest
index" fallback would drop an *unrelated* label — which does not clear the violation,
so the loop re-legalizes and cascades, shedding labels chasing an unsatisfiable floor.

**Fix:** marker clearance is a best-effort floor that **must not trigger drops**.
`legalize` gains an optional `soft::BitVector` (default `falses(n)`). Soft nodes
participate fully in constraint generation and Dykstra projection (they still *push*
labels off markers) but are **excluded from the returned `residual`** — the final
residual loop skips any pair in which either node is soft. The drop loop in
`legalize_and_drop` therefore gates on a residual reflecting only label–label and
label–obstacle penetration, exactly as today. Marker-clearance shortfalls in
over-capacity/corner cases are surfaced via `cost.jl`'s `point_overlaps` (and the
returned `residual` stays a hard label/obstacle measure), not via silent label drops.

This is the only change to `legalize.jl`: one new optional kwarg and a skip in the
final residual loop. `dykstra!` is unchanged.

### 2. `markersize` convenience attribute on `textrepel!`

New **recipe-only** attribute `markersize = nothing` (scalar `Real` or `nothing`; a
`Vector` raises a clear `ArgumentError`, matching the deferred-snooping rationale —
per-point marker sizes are out of scope). It is **not** a `RepelParams` field.

In the recipe `plot!` body, before `RepelParams` construction inside the solve `lift`,
derive the effective `point_padding`:

```
eff_point_padding = markersize === nothing ? point_padding : (markersize/2 + 0.5)
```

`markersize` (when set) overrides the `point_padding` attribute. No conflict warning:
recipe attributes always carry a value, so "user set point_padding explicitly" is not
reliably detectable, and a spurious warning would be worse than silent override —
documented instead. `markersize` is added to the solve `lift`'s dependency list so
changing it re-solves; the derived value is threaded into `RepelParams` so
`computed_params.point_padding` carries it (keeping the recipe == direct-solve guard
honest). Float coercion explicit.

**Align caveat:** the clearance guarantee assumes the rendered text box coincides with
the solver box, i.e. `align = (:center, :center)` (the recipe default). The recipe does
not apply the annotation path's `align_bias`, so under non-center `align` the rendered
box shifts off the solver box and the floor becomes approximate. Documented as a
scoped guarantee (center align); non-center align voids the strict floor.

### 3. Better default `point_padding` (decoupled from the primitive)

**Problem the review surfaced (major):** `RepelParams.point_padding` is the
ForceSolver's point-repulsion halo (`force_model.jl`), not just a connector trim.
Bumping the primitive default would silently move the fallback solver's equilibrium
and break force-model tests for no benefit (the ProjectionSolver, not the force solver,
is what needs the clearance).

**Fix — bump only the ProjectionSolver user surfaces:**

- `RepelParams.point_padding`: **stays `0.0`** (force-appropriate primitive default).
- recipe `point_padding` attribute: `2.0 → 5.0`.
- `TextRepelAlgorithm` keyword constructor: explicit `point_padding = 5.0` default
  (it forwards kwargs to `RepelParams`, so the default must be set in the ctor, not
  inherited).

`5.0` clears Makie's default `markersize = 9` (radius 4.5) with a 0.5 px gap, and
matches the `markersize=9 → m/2+0.5 = 5.0` derivation. The connector anchor-end trim
(`connectors.jl`) on the recipe path grows from 2 → 5 px — desirable, leaders tuck
further under the marker. The ForceSolver and any bare `RepelParams()` construction are
unaffected.

### 4. Annotation surface semantics

`solve_cluster` is shared, so the marker keep-out applies to **both** surfaces. On
`annotation!`, `point_padding` becomes a generic **anchor keep-out** (clearance from
the data point); this is consistent — `annotation!` users draw their own scatter the
same way recipe users do. The all-pinned bypass in `annotation_algorithm.jl` (which
skips `solve_cluster` when every label is pinned) is exempt. Pinned labels are held at
their pinned offset and never pushed off a marker (see §5). Documented in the
`TextRepelAlgorithm` docstring.

### 5. Scope of the post-legalize floor guarantee

**Problem the review surfaced (major):** `legalize` separates each pair on one axis,
and its final in-bounds clamp can pull a box back into a node it had been separated
from. So the floor cannot be claimed unconditionally.

**Guaranteed** `point_covered(anchor_j, unpadded_box_i, point_padding) == false` for
label `i` × anchor `j` **only when**: the layout is feasible (not over-capacity), the
anchor can be cleared in-bounds, `only_move = :both`, label `i` is non-pinned and
non-dropped, `align = (:center,:center)`, and the solve converged (returned
`residual ≤ 0.5`). Outside these conditions it degrades to best-effort with the
shortfall reported via `point_overlaps`/`residual` — the same posture label–label
overlap already has. **Pinned labels are exempt** (held bit-identically at their
pinned offset; never pushed, never dropped, no warning).

### 6. Documentation

Reframe `point_padding` as the marker-clearance knob in the recipe docstring
(`recipe.jl`), the `RepelParams` docstring (`params.jl`, noting it doubles as the
ForceSolver halo), the `legalize`/projection notes, the `TextRepelAlgorithm` docstring
(§4 + pinned carve-out), and CLAUDE.md (architecture section + move the point-aware
legalize note from "deferred non-goal" to "implemented; soft keep-out nodes"). Add
guidance: set `point_padding = marker_radius + small_gap`, or use `markersize`.

### 7. Performance note (minor)

Adding `n` fixed keep-out nodes grows the legalize pair scan from `O((m+k)²)` to
`O((m+k+n)²)`, repeated inside each UNCROSS swap trial. Marker-marker and
marker-obstacle pairs are skipped cheaply by the existing `(!movable[i] &&
!movable[j])` boolean guard (no `overlap_push`), so the added cost is a constant-factor
(~4×) blowup on the cheap skip check, negligible for typical `n` (tens). No
optimization in v1; a complexity note is added. Spatial pruning of each label's
keep-out set is a documented future option.

## Out of scope (deferred to a follow-up issue)

**Auto-snooping the sibling scatter's `markersize` at render time.** Investigated
against Makie 0.24 source. `Makie.parent_scene(p)::Scene` reaches the axis scene;
`scene.plots::Vector{Plot}` could be filtered for `Scatter`; default `markersize = 9`
(`theming.jl`), `markerspace = :pixel` (`basic_plots.jl`). **Disqualifying blocker:**
`scene.plots` is a plain `Vector{Plot}` mutated by a bare `push!` (`scenes.jl`) — not
an Observable. The recommended (and hero-example) layering draws `scatter!` *after*
`textrepel!`; at the moment textrepel's lift first fires the sibling does not exist,
and nothing retriggers the solve when it is later pushed. Making this work needs a
defer-to-first-render + scene-poll redesign of the recipe's reactive model. Further
problems: position→anchor matching ambiguity, `markerspace = :data` / non-disc markers
having no single radius, and private-API coupling (the risk class the annotation hook
already guards with a canary). Disproportionate to a clearance knob; filed separately.

## Affected files

- `src/legalize.jl` — add optional `soft::BitVector` kwarg; exclude soft pairs from the
  returned `residual` (§1a). The only solver-primitive change.
- `src/solvers/projection.jl` — append `n` fixed+soft anchor keep-out nodes to
  `legalize_and_drop`'s working arrays (§1); pass `soft` through to `legalize`.
- `src/params.jl` — `point_padding` default unchanged (`0.0`); reframe docstring (§6).
- `src/recipe.jl` — new `markersize` attribute; derive effective `point_padding` in the
  lift; add `markersize` to lift deps; bump `point_padding` attr `2.0 → 5.0`; reframe
  docstring; align caveat (§2, §3).
- `src/annotation_algorithm.jl` — `point_padding = 5.0` ctor default; docstring (§4).
- `CLAUDE.md` — architecture + state notes (point-aware legalize implemented, soft
  keep-out, decoupled default).
- `examples/readme_example.jl` — set `markersize = 9` so the hero documents the new
  knob; regenerate `assets/example.png`.

## Testing

TDD. Run the suite once, tee to `test/output/test-<agent-id>.log`, grep (per CLAUDE.md).

**New / revised assertions:**

1. **Clearance-floor regression** (`test_projection_solver.jl`): a *feasible*,
   `only_move=:both` fixture with anchors away from viewport edges (so the in-bounds
   clamp does not fight the keep-out). After a full solve with `residual ≤ 0.5`, assert
   the floor for every **non-dropped, non-pinned** label `i` × anchor `j` (own + foreign)
   with a tolerance consistent with legalize's `0.01` px detection — e.g. assert the
   cheaper-axis center gap `≥ unpadded_half + point_padding − 0.01`, or
   `!point_covered(anchor_j, unpadded_box_i, point_padding − 0.05)` — **not** a strict
   `point_covered == false` (which flakes at the tolerance boundary).
2. **Hero `point_overlaps == 0`** (`test_integration.jl`): the readme dataset with
   `markersize = 9` yields `solve_stats(...).point_overlaps == 0` (currently 1) **and**
   `count(dropped) ≤` the current baseline (so it cannot be met by dropping the
   offender). Name the stat source explicitly (`solve_cluster` + `label_cost` on the
   hero anchors/sizes).
3. **`markersize` attribute** (`test_integration.jl`): `markersize = m ⇒`
   `computed_params.point_padding == m/2 + 0.5`; `markersize` overrides an explicit
   `point_padding`; a `Vector` markersize raises `ArgumentError`. Re-solve test: set
   `markersize`, `update_state_before_display!`, capture offsets; change `markersize`,
   re-update, assert offsets changed (lift dependency works).
4. **Default-change fallout:** update numeric/comment assertions that shift under the
   recipe default `5.0`. Specifically re-derive and fix:
   - `test_integration.jl` "connectors suppressed when clamp pins anchor inside its own
     label" — the own-anchor keep-out changes this; decide & document the new outcome
     and fix the stale `point_padding = 2.0` rationale comment.
   - `test_annotation_algorithm.jl` warm-start invariant — correct the "default
     point_padding = 0" comment (assertion may survive; rationale must reflect 5 px).
   - Force-solver structural/crossing tests whose non-vacuity depends on seed geometry —
     pin them to an explicit `RepelParams(point_padding=...)` or verify post-change.
     (The ForceSolver default is *unchanged* at 0.0, so most force tests are unaffected;
     this guards the few that read the recipe/shared default.)
5. **`legalize.jl` soft-node unit tests** (`test_legalize.jl`): (a) a fixed node with
   **negative** half-extent separates a movable box correctly; (b) a `soft` fixed node's
   violation is **excluded** from the returned `residual` while still producing the
   separating push; (c) at `point_padding = 0` (`mc = −box_padding`, `gap =
   unpadded_half`) a clean Imhof-slot label is returned **bit-identical** (no spurious
   push), and a label legalized to cover the anchor is pushed back to the unpadded edge.
6. **Warm-start / pinned / axis-lock / over-capacity** (`test_projection_solver.jl`):
   - warm-start: `solve_cluster` with `init_state` placing a label on its own anchor →
     pushed to clear `point_padding`.
   - pinned: a pinned label whose `pinned_offset` covers its own marker is returned
     bit-identically (not pushed, not dropped, no warning).
   - `only_move = :x` with a foreign marker overlapping only on `y` → graceful
     (floor waived on the locked axis; no unexpected drop).
   - over-capacity / corner anchor with in-bounds-unsatisfiable clearance → solve
     terminates within existing caps, stays label-overlap-free, reports a sane dropped
     count, and does **not** cascade-drop on the marker residual (validates §1a).
7. Regenerate `assets/example.png`; eyeball: no marker-occluded labels.

## Acceptance (from the issue, with scope from §5)

- A non-pinned, non-dropped label's text box clears every marker disc (own and
  foreign) by ≥ the configured clearance **after** legalize, for feasible
  `only_move=:both` center-align layouts. ✅ (§1, test 1)
- `point_overlaps == 0` on the README/hero dataset (currently 1), without extra drops.
  ✅ (test 2)
- Regenerated hero image shows no marker-occluded labels. ✅ (test 7)
- New regression test on a dense fixture asserting the post-legalize clearance floor.
  ✅ (test 1)
- Marker clearance never silently drops labels (best-effort floor; drops remain gated
  on label–label/obstacle overlap). ✅ (§1a, test 6)
