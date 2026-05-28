# Own-Point Repulsion & Connector Geometry Hardening — Design

**Date:** 2026-05-27
**Status:** Approved design, pre-implementation (revised after 3-reviewer pass)
**Follows:** `2026-05-27-axis-clamping-design.md` (and the base package design)
**Closes:** [jowch/MakieTextRepel.jl#1](https://github.com/jowch/MakieTextRepel.jl/issues/1)

## Goal

Stop labels from sitting on top of their own data markers, and stop the leader
lines that connect labels to their anchors from rendering as visual jank — lines
drawn through the label, tiny stubs poking out of glyphs, lines disappearing
under scatter markers. In the v0.1 README hero (`assets/example.png`) and the CI
smoke fixture (`test/output/demo.png`), several labels (`node 3`, `node 11`,
`alpha`, `epsilon`, …) sit directly on their markers with no visible connector
and the connector geometry has degenerate edge cases visible to the eye.

## Background (from investigation, 2026-05-27)

The visible artifacts trace back to one root cause plus two latent geometry
bugs in the connector layer.

1. **The solver lets labels settle on their own anchor.** `solve_repel`
   deliberately skips repelling each label from its own anchor in the
   point-push loop (`src/solver.jl:109`, `i == j && continue` inside the
   `point_push` loop — *not* the overlap-push skip on line 104, which stays),
   and `explode_init` only fans out the initial offsets when anchors are
   *coincident* (`src/solver.jl:26-42`, threshold `1e-3`). An isolated label
   therefore starts at offset `(0, 0)` with no own-anchor force pushing it
   away. The weak default `force_pull = 0.01` provides no escape vector.
   Result: the label settles on its anchor. This is issue #1.

2. **`clip_to_box_edge` produces a past-the-target endpoint when the target
   is inside the box.** `src/geometry.jl:51-61` computes
   `t = min(hw/|d_x|, hh/|d_y|)` and returns `c + t·d`, but never clamps `t`
   to `[0, 1]`. When the anchor lies inside the padded label box, `t > 1`
   and the returned point sits past the anchor on the far side of the box.
   The drawn segment then travels from the anchor (inside the box) *through
   the text* and out the opposite face. This is the "tiny stub poking out
   of `epsilon`" artifact in `test/output/demo.png`.

3. **`min_segment_length` filters offset magnitude, not visible segment
   length.** `src/connectors.jl:15` uses `norm(offsets[i]) <= min_len`
   (default `5.0` px). Typical label half-widths plus padding are 20–40 px,
   so offsets in `(5, ~40]` px draw a connector while the anchor is still
   deep inside the padded box. The 5-px filter does not catch this regime.

4. **The connector's anchor end is never trimmed.** `build_connectors`
   pushes `(anchor, edge)` — the line runs all the way to the data point.
   With a scatter marker at that point, the line disappears under the
   marker. `point_padding` is consumed by the solver (`src/solver.jl:110`)
   but not by the connector layer.

The own-anchor skip was intentional: a deterministic solver settling a
label *exactly* on its anchor has a zero-length repulsion gradient, so the
anisotropic push falls back to a fixed `+x` direction, which would bias
every isolated label rightward. ggrepel avoids this with random init jitter.
We need a deterministic equivalent.

## Decision

Two-layer fix.

- **Solver layer:** give every label a deterministic per-label initial
  offset on a golden-angle spiral, sized to escape its own padded box, and
  drop the own-anchor repulsion skip in the point-push loop. Determinism
  preserved; no random jitter introduced.
- **Connector layer:** fix `clip_to_box_edge` to refuse to draw when the
  target is *strictly inside* the box, trim the anchor end of each segment
  by `point_padding`, and clamp `t` to `[0, 1]` defensively. The connector
  layer now has invariants that make the J1/J2 visual jank impossible *as
  geometry*, independent of solver behavior.

Defaults: bump `point_padding` from `0.0` to `2.0` px so the leader-to-marker
gap is visible at the package's default scatter sizes (`markersize ≈ 9` at
the common DPI), and drop `min_segment_length` from `5.0` to `2.0` px to
re-tune the filter for its new visible-length semantics (5 px of *visible*
line is a much stronger filter than 5 px of offset magnitude; 2 px still
suppresses sub-pixel jitter without eating normal short leaders).

## Architecture

### Solver (`src/solver.jl`)

Replace `explode_init` with a more general `init_offsets`. Every label `i`
gets a deterministic offset:

- **Angle.** `θᵢ = i · φ_g` where `φ_g = π · (3 − √5) ≈ 2.39996` rad
  (golden angle, standard for phyllotaxis spirals — gives the most
  spread-out indexing on a circle). Starting at `i · φ_g` (not
  `(i − 1) · φ_g`) keeps the single-label case off the cardinal x-axis
  (`θ₁ ≈ 137°`, not `0°`); a small-but-real readability improvement and
  it removes the "wait, is this the +x bias the spec is supposed to
  avoid?" question.
- **Magnitude.** `rᵢ = max(√((hwᵢ + pad)² + (hhᵢ + pad)²), 1.0)` where
  `hwᵢ, hhᵢ` are the label's half-width and half-height and
  `pad = box_padding`. The unfloored value is the corner-distance of the
  padded box; placing the offset at distance `rᵢ` from the box center
  along *any* angle puts the anchor *on or outside* the padded box (the
  box's corner is at distance `rᵢ` from the center, so along any direction
  the box face is no farther than `rᵢ`). The `1.0` px is a
  degenerate-input floor (zero-size label *and* zero `box_padding`,
  e.g. empty strings) — *not* a normal-case escape-distance constant. At
  the default `box_padding = 4.0` the bare formula gives `rᵢ ≥ 4√2 ≈ 5.66`,
  well above the floor; the floor never binds in normal layouts.
- **Offset.** `offsetᵢ = (rᵢ cos θᵢ, rᵢ sin θᵢ)`.

This subsumes the coincident-anchor case (each label in a cluster still
gets a distinct angle) and the isolated-label case (each gets a distinct
escape vector), with one rule.

**Determinism scope.** The angle is a pure function of index; magnitude
is a pure function of label size and `box_padding`. Same input → same
output. The angle is *not* stable across label insertion/deletion — a
user who adds a label at index 1 shifts every subsequent label's seat on
the spiral. This matches ggrepel's behavior (which re-randomizes the
whole layout on each call); stability-across-edits via `hash(text)` or
`hash(anchor)` was considered and rejected (adds a dependency, has its
own tie-breaking story for repeated labels and coincident anchors, and
solves a problem ggrepel itself doesn't solve).

**Why the corner-distance formula is exactly right.** Along any direction
`(cos θ, sin θ)`, the padded box's face in that direction is at distance
`min(hw_p/|cos θ|, hh_p/|sin θ|)` from the center (where `hw_p = hw + pad`,
`hh_p = hh + pad`). The maximum of that over `θ` — the worst-case escape
distance — is reached at the corner directions, where it equals
`√(hw_p² + hh_p²)`. Picking `rᵢ` at this corner-distance means the anchor
is *outside the padded box for almost all angles* and *exactly on a corner*
for the cardinal-corner angles. The connector layer's strict-inside check
(below) treats boundary points as valid endpoints, so this is fine.

**Drop the own-anchor skip in `point_push`** (`src/solver.jl:109`,
inside the point-push loop only — the overlap-push skip on line 104
stays; labels still don't push themselves out via box-overlap). Every
label now feels a repulsion from its own anchor with the same
`force_point` strength as from every other anchor (matches ggrepel — the
kernel has no notion of "home"). The existing `force_pull` spring brings
labels back toward the anchor; equilibrium between own-anchor push and
spring settles each label *near* (not *on*) its point.

**Equilibrium mechanics.** `point_push` (`src/geometry.jl:39-45`)
internally inflates the box by `point_padding` before testing whether the
anchor lies inside it — its influence region is the *padded* box further
inflated by `point_padding`. At init, the anchor sits on the padded-box
corner, which is *inside* the inflated influence region by up to
`point_padding`; own-anchor push is therefore small but nonzero at init
(zero exactly along the corner ray, weak along nearby directions). The
spring is the dominant inward force and drives the label toward its
anchor; equilibrium is reached when the growing own-anchor push (as the
anchor crosses deeper into the influence region) balances the spring.
With default `force_pull = 0.01` and `pull_threshold = 1.0`, equilibrium
sits at `d ≈ hw + box_padding + point_padding` from the anchor along the
dominant axis — the anchor lies just inside the influence boundary.
Raising `force_pull` materially could collapse this equilibrium and
re-introduce labels-on-points; the defaults assume the current ratio.

No new `RepelParams` field is required; `box_padding` and `point_padding`
are already in scope.

**Interaction with `clamp_box_offset`.** A label clamped against an axis
edge can end up with its anchor inside its own padded box, because the
clamp moves the *box* but the anchor is fixed at a data position. If the
anchor's projected position falls inside the clamped range of the box,
the clamp cannot push it back out without flipping the label across the
anchor — a much bigger semantic change, out of scope. The connector
layer suppresses the segment cleanly in this case (the label still
renders, just without a leader). This matches ggrepel's behavior when
`put_within_bounds` collapses a label onto its anchor.

### Geometry (`src/geometry.jl`)

Change `clip_to_box_edge`'s contract:

```julia
clip_to_box_edge(box::Rect2f, target::Point2f) -> Union{Point2f, Nothing}
```

Behavior:
- If `target` is **strictly** inside `box` on both axes
  (`|d_x| < hw && |d_y| < hh`) → return `nothing`. The caller suppresses
  the segment. (Strict inequality: a target on a face or corner is a
  *valid* connector endpoint at `t = 1.0` — see below.)
- Otherwise compute `t = min(hw/|d_x|, hh/|d_y|)` as today; **clamp
  `t` to `[0, 1]`** defensively. The strict-inside check above already
  guards against `t > 1`, but the clamp locks the invariant into the
  function.
- The existing `d == (0, 0)` early-return is now covered by the
  strict-inside branch (`|0| < hw` and `|0| < hh` for any non-degenerate
  box).

This is a behavior change for callers, but `clip_to_box_edge` has exactly
one caller in the package (`build_connectors`), so the change is local.
No public API impact.

### Connectors (`src/connectors.jl`)

`build_connectors` gains `point_padding` as a **keyword argument with
default `0.0`** (not positional — keeps the recipe's existing call site
compiling between the connector change and the recipe wiring change, so
intermediate builds in the sequence stay green):

```julia
build_connectors(anchors, offsets, sizes, dropped,
                 min_segment_length, box_padding;
                 point_padding = 0.0) -> Vector{Point2f}
```

Per-label loop:
1. Skip dropped labels.
2. Compute `box = box_at(anchor, offset, size + 2·box_padding)`.
3. Call `edge = clip_to_box_edge(box, anchor)`. If `edge === nothing`,
   skip the segment (anchor was strictly inside the padded box).
4. Compute `dir = edge - anchor`. If `‖dir‖ ≤ point_padding`, skip the
   segment (anchor trim would invert the direction). This case is
   pathological in practice; the strict-inside check at step 3 already
   covers most of it, but pixel-fractional jitter near the boundary
   needs this guard.
5. Compute `û = dir / ‖dir‖`, then `seg_start = anchor + point_padding · û`.
6. **Apply `min_segment_length` to the visible segment length:**
   suppress when `‖edge − seg_start‖ ≤ min_segment_length` (matches the
   existing `<=` convention from `connectors.jl:15`). This fixes J3:
   the filter now operates on what the user sees, not on offset
   magnitude.
7. Push `(seg_start, edge)`.

**Filter redundancy is intentional.** Steps 4 and 6 are both lower
bounds on visible length (step 4 prevents direction inversion; step 6
suppresses too-short leaders). With `point_padding ≤ min_segment_length`
(the normal case where padding is 2 and the filter is 2), step 6 is
binding; step 4 only fires for pathological values where padding is
larger than the filter.

Result: connectors never enter the label, never disappear under the
marker, and the minimum-length filter governs what the user actually
sees.

### Recipe (`src/recipe.jl`)

- Default `point_padding = 2.0` (was `0.0`). Update the attribute
  docstring to describe both the solver halo and the connector anchor
  gap. *Note:* the recipe has no access to `markersize` (that comes from
  a sibling `scatter!` call, not the recipe's own observables), so
  `point_padding` is a fixed-pixel default — users with unusually small
  or large markers should override it.
- Default `min_segment_length = 2.0` (was `5.0`). Re-tuned for the new
  visible-length filter semantics; documented in the docstring.
- Forward `point_padding` into the `build_connectors` lift (currently
  only `box_padding` is forwarded).
- No change to plot order. The geometry fix removes the only case where
  z-order produced visible jank (connectors entering the box); reorder
  is out of scope.

## Testing

The testing posture is **intentional behavior change**, not "no
semantics changes" — the whole point of this work is to fix observed
jank. Two existing tests assert behaviors that are *now wrong* and will
be inverted as part of this work (they are the regression tests for the
fix); a handful of others have numeric expectations that depend on the
old `explode_init` magnitude and will be updated to match the new
corner-distance formula, with the *intent* preserved.

### Existing tests that flip (these encode the bug)
- `test/test_solver.jl:54` — single-label `o1 == [Vec2f(0, 0)]` flips:
  isolated labels now move under own-anchor repulsion.
- Any assertion that locks the old `(w+h)/4` magnitude on coincident
  anchors (audit during implementation).

### `test/test_geometry.jl`
- `clip_to_box_edge` returns `nothing` when target is strictly inside
  box.
- `clip_to_box_edge` returns `nothing` at the box center (`d = 0`).
- `clip_to_box_edge` returns a valid face point when target sits on a
  face (`|d_x| = hw, |d_y| < hh`) — `t = 1`, face endpoint.
- `clip_to_box_edge` returns a valid corner point when target sits on a
  corner (`|d_x| = hw, |d_y| = hh`) — `t = 1`, corner endpoint.
- Clamp behavior: a target just outside the box returns a point on the
  near face, not past the target.

### `test/test_solver.jl`
- `init_offsets` is deterministic per index and label size.
- For every label (including zero-size and ordinary cases), the anchor
  at `box_at(anchor, offsets[i], psize)` lies *on or outside* the padded
  box (geometric invariant; the `1.0` px floor keeps zero-size labels
  honest).
- Two distinct labels at the same anchor get distinct golden-angle
  offsets (replaces the existing coincident-fan-out test).
- Cluster stress: 5+ labels at the same anchor with varying sizes — all
  finite, all pairwise non-overlapping after `solve_repel`.
- An isolated single label settles with `0 < ‖offset‖ < init_r` after
  `solve_repel` (own-anchor repulsion active; spring pulled it inward
  from the init).
- Zero-size label (empty string): non-zero offset after `solve_repel`
  (regression for the `r_min` floor).
- Determinism on a non-degenerate input: same input → byte-identical
  output (regression for the new force term).

### `test/test_connectors.jl`
- Segment start lies `point_padding` from the anchor along the segment
  direction.
- Anchor inside the padded box: segment list is empty (or shorter by
  the expected number of suppressed entries when mixed with normal
  labels).
- Visible-length filter at the boundary: anchor-to-edge of
  `min_segment_length + point_padding` exactly suppresses (`<=`); one
  unit larger emits a segment.
- Diagonal offset: segment terminates at the appropriate box face/corner
  (regression for the single-horizontal-case coverage today).
- Multiple labels at the same anchor produce fanned-out segments in
  distinct directions (no uniform-rightward bias).

### `test/test_integration.jl`
- Re-render the smoke demo; visually verify (eye-pass on
  `test/output/demo.png`) that no label sits on its marker and no
  connector enters its label.
- Existing axis-clamp integration test still passes.
- **Adversarial clamp case:** label wider than half the viewport,
  anchored near a viewport corner. After clamp, the connector layer
  suppresses the segment; no segment renders through the label; the
  test reads the segment array and asserts it omits the clamped label.

## Backward compatibility

- v0.1 is unreleased (per `MEMORY.md`); default changes are acceptable.
- Users who set `point_padding` or `min_segment_length` explicitly are
  unaffected by the default bumps.
- The own-anchor repulsion is intentional and not opt-out — that *is*
  the fix for #1.
- **`force_pull = (∞, ∞)`** simulates the old "labels on points"
  behavior (spring overwhelms own-anchor push).
- **`force_pull = (0, 0)`** is the symmetric extreme: with no spring,
  own-anchor push drives every label outward until the viewport clamp
  catches it. Labels pin to viewport edges; figures remain finite. Users
  who explicitly want this should know what they're getting.
- `clip_to_box_edge`'s return-type change is
  `Point2f → Union{Point2f, Nothing}`. Public-API impact: none (it is an
  internal helper; the only caller is `build_connectors`).
- `build_connectors`'s signature gains `point_padding` as a *keyword*
  argument with default `0.0`, so existing callers don't break between
  build steps.

## Build sequence

Each step is independently mergeable — intermediate states compile and
their tests pass.

1. **Geometry + minimal connector absorption.** Update
   `clip_to_box_edge` to return `Union{Point2f, Nothing}` and add the
   strict-inside / t-clamp logic. In the same step, update
   `build_connectors` to skip with `edge === nothing && continue` so it
   absorbs the new return type without crashing on `push!` of `nothing`.
   Add geometry unit tests. (Two files touched, but the connector
   change is one line; keeping them together is what makes Step 1
   actually mergeable.)
2. **Connectors hardening.** Add `point_padding` as a keyword argument
   (default `0.0`), add the anchor trim, switch the
   `min_segment_length` filter to the visible-length test. Add
   connector unit tests. Recipe still passes only `box_padding`; the
   keyword default keeps behavior identical.
3. **Solver.** Introduce `init_offsets` (with `r_min` floor), remove
   the own-anchor skip in `point_push` line 109. Add solver tests and
   update/invert the existing tests that asserted the buggy behavior.
4. **Recipe + defaults.** Wire `point_padding` through to
   `build_connectors`, bump `point_padding` default to `2.0`, bump
   `min_segment_length` default to `2.0`. Regenerate the smoke-test
   demo image and the README hero. Visual verification (eye-pass).

Steps 1 and 3 are the substantive changes; 2 and 4 are wiring.

## Out of scope

- **Plot order rework** (drawing connectors before text/box). The
  geometry fix removes the only visible jank caused by z-order; reorder
  is unnecessary and risks breaking other consumers of the recipe.
- **`force_pull` / `pull_threshold` defaults.** The "close but no
  connector" gap in the (1, 5] px offset band disappears in practice
  once init places labels well outside their box and own-anchor
  repulsion keeps them there. Defaults can be revisited if real layouts
  still show the gap.
- **Custom anchor markers.** A "small filled dot at anchor instead of a
  line when no clean segment is possible" treatment was considered and
  deferred — the segment-suppression policy is simpler and matches
  ggrepel.
- **Per-label `point_padding`.** Single global value, matching the
  existing `box_padding` style.
- **Stability across label insertion/deletion.** Golden-angle indexing
  is by position, so adding/removing labels shifts the whole spiral.
  Matches ggrepel's default behavior; a hash-based stable indexing is
  not built.
- **Label flipping across the anchor under heavy clamping.** When the
  clamp pins a wide label such that its anchor falls inside the clamped
  box, the connector is suppressed. Re-anchoring the label on the
  opposite side of the data point would preserve the leader, but it's
  a much larger semantic change (the label's quadrant relative to the
  anchor changes mid-layout) and is deferred.
- **NaN/Inf in `anchors` or `sizes`.** Treated as caller error; the
  solver propagates NaN as it does today. A defensive pre-pass is not
  added.
