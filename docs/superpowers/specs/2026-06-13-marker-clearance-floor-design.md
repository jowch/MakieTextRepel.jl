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

**Node-ordering contract (determinism).** Append the `n` anchor keep-out nodes in
**ascending anchor index, after the label and obstacle nodes, every round** — the full
anchor list independent of the current dropped set. This pins Dykstra's projection
order, keeping the solve deterministic and round-invariant. A future refactor must not
reorder these appends.

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

New **recipe-only** attribute `markersize = nothing` (scalar `Real` or `nothing`). It
is **not** a `RepelParams` field. **`textrepel!` draws no markers** — this attribute
only declares the size of the *sibling* `scatter!` marker so the solver can clear it;
the docstring's first sentence must say so plainly (the name mirrors Makie's scatter
attribute for discoverability, but the recipe never renders a marker itself). The
`m/2 + 0.5` radius→clearance derivation assumes a **disc marker in
`markerspace = :pixel`** (the scatter default); for non-disc or `:data`-space markers,
set `point_padding` manually (documented). A `Vector` `markersize` raises a clear
`ArgumentError` whose message names the attribute — note this fires on **first
solve/display** (inside the lift), not at construction, since the recipe validates in
the solve path; documented.

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

The floor is a **per-axis-disjunctive** clearance: `legalize` separates each
overlapping pair on a single (cheaper) axis, while `point_covered` is a two-axis AND
(false as soon as the anchor is outside the expanded box on *either* axis). So the
achievable invariant is "cleared on at least one axis to ≥ `unpadded_half +
point_padding`", which makes `point_covered(...) == false`. It is **not** a two-axis
center-gap guarantee.

**Guaranteed** `point_covered(anchor_j, unpadded_box_i, point_padding − τ) == false`
(for a small `τ ≈ 0.01` px = Dykstra detection tol + Float32 noise) for label `i` ×
anchor `j` **only when**: the layout is feasible (the anchor can be cleared in-bounds),
`only_move = :both`, label `i` is non-pinned and non-dropped, `align = (:center,:center)`,
the label–marker constraint was not overridden by a competing label–label constraint in
the same Dykstra pass, and the solve converged. **This floor is NOT witnessed by the
returned `residual`** — by §1a the soft keep-out nodes are excluded from `residual`, so
`residual ≤ 0.5` says nothing about marker clearance. Marker-clearance shortfall is
surfaced only via `cost.jl`'s `point_overlaps`. Outside these conditions the floor
degrades to best-effort (reported via `point_overlaps`), the same posture label–label
overlap has. **Pinned labels are exempt** (held bit-identically at their pinned offset;
never pushed, never dropped, no warning).

### 6. Documentation

Reframe `point_padding` as the marker-clearance knob in the recipe docstring
(`recipe.jl`), the `RepelParams` docstring (`params.jl`, noting it doubles as the
ForceSolver halo), the `legalize`/projection notes, the `TextRepelAlgorithm` docstring
(§4 + pinned carve-out), and CLAUDE.md (architecture section + move the point-aware
legalize note from "deferred non-goal" to "implemented; soft keep-out nodes"). Add
guidance: set `point_padding = marker_radius + small_gap`, or use `markersize`.

### 7. Performance note (minor)

Adding `n` fixed keep-out nodes grows the legalize pair scan from `O((m+k)²)` to
`O((m+k+n)²)` per round, repeated inside each UNCROSS swap trial. Marker-marker and
marker-obstacle pairs are skipped cheaply by the existing `(!movable[i] &&
!movable[j])` boolean guard, but the **`n × m` label-vs-marker pairs are real
constraint evaluations** (overlap detection + possible Dykstra constraint), not skips —
that is the actual added cost. Cheap for typical `n` (tens); no optimization in v1, with
a complexity note added. Spatial pruning of each label's keep-out set is a documented
future option.

**Round-cap interaction:** markers never *force* extra rounds in the common case (a
satisfied soft pair adds no constraint, so the no-constraint early break still fires).
But a corner/edge anchor whose clearance is in-bounds-unsatisfiable can cycle — the
final in-bounds clamp re-introduces the penetration the keep-out re-pushes — so such
scenes may run legalize to the full `rounds × iters` budget × swap trials. Accepted as
a pathological-input cost (the same backstop regime as box over-capacity); noted in §5.

## Out of scope (deferred to a follow-up issue)

**Auto-snooping the sibling scatter's `markersize` at render time.** Investigated
against Makie 0.24 source. `Makie.parent_scene(p)::Scene` reaches the axis scene;
`scene.plots::Vector{Plot}` could be filtered for `Scatter`; default `markersize = 9`
(`theming.jl`), `markerspace = :pixel` (`basic_plots.jl`). **Disqualifying blocker:**
`scene.plots` is a plain `Vector{Plot}` mutated by a bare `push!` (`scenes.jl`) — not
itself an Observable. The recommended (and hero-example) layering draws `scatter!`
*after* `textrepel!`; at the moment textrepel's lift first fires the sibling does not
exist. A retrigger hook **does** exist — `events(scene).tick::Observable{Tick}` is a
per-frame Observable an `on()` callback could use to defer marker discovery to first
render and re-solve — so the deferral does **not** rest on "no possible trigger."
Rather, even with the tick hook the approach remains brittle: it forces a
defer-to-first-render + per-frame scene-poll into the recipe's otherwise-declarative
reactive model, and still faces position→anchor matching ambiguity, `markerspace =
:data` / non-disc markers having no single radius, and private-API coupling (the risk
class the annotation hook already guards with a canary). Disproportionate to a clearance
knob; filed separately.

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
   clamp does not fight the keep-out). **TDD-red precondition:** the fixture must be a
   "legalize-erasure" case — `init`/`side_select` place a clean slot but legalize shoves
   a box back over a marker — and must be verified **failing against the pre-keep-out
   solver** (capture the current `point_covered` violation) before the fix; otherwise
   the test is vacuous (the existing "Q battery" already asserts `point_overlaps == 0`
   at `point_padding = 2.0` without the keep-out, so a naive fixture would pass today).
   After a full solve, assert **only the disjunctive predicate**: for every
   **non-dropped, non-pinned** label `i` × anchor `j` (own + foreign),
   `!point_covered(anchor_j, unpadded_box_i, point_padding − 0.05)` (the `0.05` eps
   absorbs legalize's `0.01` detection tol + Float32 noise). **Do not** assert a
   per-axis center gap — legalize separates on one axis only, so the floor is per-axis
   disjunctive (see §5), and a center-gap assertion fails on valid layouts.
2. **Hero `point_overlaps == 0`, no extra drops** (`test_integration.jl`, net-new
   infrastructure): extract the hero dataset into a **shared deterministic source**
   (seeded `hero_dataset()`, copied verbatim into the test — tests must not `include` the
   example) so the test scores the same scene as the hero image. **The test must render
   the COMMITTED artifact geometry**, not a convenient stand-in: build the 2100×700
   three-panel figure (three `aspect=1` titled axes; `textrepel!` on the middle) and read
   `computed_*`/`label_cost` off the middle panel. (A single 400×400 axis is a *different*
   pixel viewport that drops a different label count and would mask extra drops on the
   real artifact — PR-review round 1 caught exactly this.) Assert **`count(dropped) == 0`
   AND `point_overlaps == 0`** — the honest acceptance bar. The hero figure is sized
   2100×700 specifically so the 5 px floor has room to clear every marker with zero drops;
   at the old 1200×400 the middle panel was over-capacity and dropped node 11 + node 19.
   Stat source: `label_cost` on the middle panel's `computed_*`.
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
     **Sweep all `TextRepelAlgorithm()` / kwarg-ctor call sites** in that file, re-run
     under the new `pp = 5.0` ctor default + point-aware legalize, and confirm each
     inequality/in-bounds assertion still holds.
   - `test_integration.jl` "connectors suppressed when clamp pins anchor inside its own
     label" — this fixture's outcome **inverts** under the bump + own-anchor keep-out.
     The implementer must **re-derive empirically during TDD** whether the 100×80
     viewport with a ~154 px label is over-capacity (floor waived ⇒ anchor still inside
     ⇒ suppression preserved) or feasible (anchor pushed clear ⇒ connector now drawn),
     then update the assertion **and** fix the stale `point_padding = 2.0` rationale
     comment. Document the decided outcome.
   - Force-solver structural/crossing tests whose non-vacuity depends on seed geometry —
     pin them to an explicit `RepelParams(point_padding=...)` or verify post-change.
     (The ForceSolver default is *unchanged* at 0.0, so most force tests are unaffected;
     this guards the few that read the recipe/shared default.)
5. **`legalize.jl` soft-node unit tests** (`test_legalize.jl`): (a) a fixed node with
   **negative** half-extent separates a movable box correctly; (b) **on the same pair**,
   force an in-bounds-unsatisfiable soft-marker separation (label clamped to a bound,
   marker placed so full clearance is impossible): assert the offset **changed** (push
   fired) **and** the returned `residual ≤ 0.5` (soft pair excluded) — plus a **negative
   control**: the identical fixture with the node marked non-soft returns `residual > 0.5`
   (proving the exclusion is what suppresses it); (c) at `point_padding = 0`
   (`mc = −box_padding`, `gap = unpadded_half`) a clean Imhof-slot label is returned
   **bit-identical** (no spurious push), and a label legalized to cover the anchor is
   pushed back to the unpadded edge.
6. **Warm-start / pinned / axis-lock / over-capacity** (`test_projection_solver.jl`):
   - warm-start: `solve_cluster` with `init_state` placing a label on its own anchor →
     pushed to clear `point_padding`.
   - pinned: a pinned label whose `pinned_offset` covers its own marker is returned
     bit-identically (not pushed, not dropped, no warning).
   - `only_move = :x` with a foreign marker covering the box on **both** axes → assert
     the floor is achieved on x (`point_covered` false because the box cleared on the
     unlocked x axis) and **no drop** occurred. (Optionally keep a y-only case labelled
     as the expected no-op/best-effort path.)
   - **Anti-cascade differential (validates §1a):** a fixture that drops **zero** labels
     against the pre-keep-out solver, with a corner anchor whose marker clearance is
     in-bounds-unsatisfiable, must **still drop zero** with the keep-out — proving the
     soft nodes never inflate the drop-triggering residual. Assert termination within
     the existing caps and label-overlap-free output.
   - **Dropped-anchor keep-out participates:** in an over-capacity scene, a surviving
     label adjacent to a *dropped* label's anchor still clears that anchor's floor
     (confirms keep-out nodes cover the full anchor list, not just active labels).
7. Regenerate `assets/example.png`; eyeball: no marker-occluded labels.

## Acceptance (from the issue, with scope from §5)

- A non-pinned, non-dropped label's text box clears every marker disc (own and
  foreign) by ≥ the configured clearance **after** legalize, for feasible
  `only_move=:both` center-align layouts. ✅ (§1, test 1)
- `point_overlaps == 0` on the README/hero dataset (currently 1), without extra drops.
  ✅ (test 2) — verified on the committed 2100×700 three-panel artifact: 0 drops, 0
  occlusion. (The hero figure was enlarged from 1200×400 to give the 5 px floor room;
  at 1200×400 the middle panel was over-capacity and dropped node 11 + node 19, which is
  the designed over-capacity behavior, not a clean win — so the showcase uses a size
  where all 22 labels are retained.)
- Regenerated hero image shows no marker-occluded labels. ✅ (test 7)
- New regression test on a dense fixture asserting the post-legalize clearance floor.
  ✅ (test 1)
- Marker clearance never silently drops labels (best-effort floor; drops remain gated
  on label–label/obstacle overlap). ✅ (§1a, test 6)
