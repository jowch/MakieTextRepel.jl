# Marker-clearance floor: point-aware legalize (issue #21)

**Date:** 2026-06-13
**Issue:** #21 — Marker occlusion: enforce a marker-clearance floor (point-aware legalize)
**Status:** Design approved; ready for implementation plan.

## Problem

Scatter markers paint over the start of label text. In the hero image
(`assets/example.png`), `node 11`'s box is covered by `node 8`'s marker — a real
foreign overlap (`point_overlaps = 1`) — and labels more generally clip *under*
their own markers across the figure. This is **occlusion**, not leader length:
the labels can sit as close to their anchors as we like; the marker glyph simply
paints over the text.

### Root cause (verified against the code)

1. **`legalize` does not treat the anchor as a keep-out.** Its working nodes are
   active label boxes ∪ obstacle pseudo-nodes only — anchors are never nodes
   (`solvers/projection.jl`, `legalize_and_drop`). So even after `init`/`side_select`
   place a box at `point_padding` from the marker (`slot_offset` in `init.jl`),
   `legalize` is free to shove the box back across its own or a foreign anchor.
   This is the "legalize-erasure" case: `side_select` clears markers; `legalize`
   silently re-covers one.
2. **The swap-uncross loop never re-checks it.** That loop (`solvers/projection.jl`)
   is gated on `find_crossings`; here `crossings = 0`, so it never runs despite
   `point_overlaps` being a term in `swapkey`.
3. **`point_padding` was smaller than a typical marker.** Recipe default
   `point_padding = 2.0` vs. Makie's default scatter `markersize = 9` (radius 4.5),
   so the marker overlaps the box by ~2.5 px even in a clean placement.
4. **`textrepel!` and `scatter!` are separate plots,** so the recipe never sees the
   externally-drawn marker size and cannot auto-clear markers unless told.

## Solution overview

One mechanism resolves the occlusion; two smaller changes make it usable and good
by default.

### 1. Point-aware legalize (the core fix)

In `solvers/projection.jl`'s `legalize_and_drop`, add **every anchor** (all `n`,
own and foreign, regardless of dropped/pinned state — the scatter markers are drawn
for every point) as a **fixed keep-out node**:

- position: the anchor point (offset `(0,0)`),
- half-extent on both axes: `mc = point_padding − box_padding` (signed; may be
  negative),
- `fixed = true` (contributes extents, never moves; not clamped to bounds).

These nodes are appended to the legalize working arrays exactly like the existing
obstacle pseudo-nodes. **No change to `legalize.jl` is required** — it treats fixed
nodes purely numerically (`hw = psize/2`), and its `ox/oy` overlap detection and
Dykstra projection handle a negative `mc` correctly. Marker-vs-marker and
marker-vs-obstacle pairs are two-fixed-node pairs and are already skipped.

**Why `mc = point_padding − box_padding` is exactly right.** `legalize` separates a
pair to a center-to-center gap of `hw_i + hw_j`. A label node carries the *padded*
half-extent `unpadded_half + box_padding`. So a label-vs-marker pair separates to:

```
gap = (unpadded_half + box_padding) + (point_padding − box_padding)
    =  unpadded_half + point_padding
```

i.e. the **unpadded text edge clears the marker center by exactly `point_padding`**,
independent of `box_padding`, for every value including 0. This is identical to what
`point_covered(anchor, unpadded_box, point_padding)` tests in `geometry.jl` (the
predicate shared by `side_select`'s marker term and `cost.jl`'s `point_overlaps`), so
the geometric floor and the Q scoreboard count the same event. `legalize` separates
each pair on its cheaper axis only; AABB-disjointness on one axis is sufficient to make
`point_covered` false (which requires the point inside on *both* axes).

**Degradation at `point_padding = 0`:** `mc = −box_padding`, so `gap = unpadded_half`.
A clean Imhof slot sits exactly on that boundary (`ox = 0`, below the `0.01`
detection threshold) → no spurious push. Behavior matches the pre-#21 layout except
that a label is now prevented from being legalized so far inward that its *unpadded*
box covers an anchor — a strict improvement, rarely triggered.

**Own-marker handling (per the issue).** The geometric floor applies to own and
foreign anchors alike (every anchor gets a keep-out node). The `j == i` skip stays in
**scoring only** — `side_select`'s `ov`/`global_key` `hard` term, `cost.jl`'s
`point_overlaps`, and `drop_most_overlapped!` — so own-marker coverage is never *counted*
as a droppable overlap. Documented edge: in a genuinely over-capacity scene a residual
that includes a marker-clearance violation can trigger the existing geometric drop
loop. Own-marker-only violations are effectively impossible for a non-degenerate solve
(a label has eight Imhof slots plus legalize freedom to clear its own anchor), so in
practice only foreign-marker clearance in an already over-capacity scene can lead to a
drop — consistent with how box-overlap over-capacity is handled today.

### 2. `markersize` convenience attribute on `textrepel!`

New recipe attribute `markersize = nothing` (default). When set to a number `m`,
the recipe derives `point_padding = m/2 + 1.0` (radius + 1 px gap). Documented as
sugar over `point_padding`. If the user sets **both** `markersize` and an explicit
`point_padding`, `markersize` wins and a one-time `@warn` (`maxlog = 1`) fires.
`markersize` is added to the recipe's solve `lift` dependency list so changing it
re-solves.

This is the honest, deterministic answer to "the recipe can't see the sibling
scatter." Automatic scene-graph snooping is explicitly **out of scope** (see below).

### 3. Better default `point_padding`

Bump the default from `2.0` to **`5.0`** — clears Makie's default `markersize = 9`
(radius 4.5) with a 0.5 px gap out of the box. Two source-of-truth changes keep all
surfaces aligned:

- `RepelParams.point_padding`: `0.0 → 5.0` (so `TextRepelAlgorithm`, which forwards
  kwargs to `RepelParams`, inherits the new default).
- recipe `point_padding` attribute: `2.0 → 5.0`.

`point_padding` also controls the connector anchor-end trim (`connectors.jl`); the leader
lines will now start 5 px from the marker instead of 2 px — desirable, leaders tuck
further under the marker.

### 4. Documentation

Reframe `point_padding` as the **marker-clearance** knob in the recipe docstring
(`recipe.jl`), the `RepelParams` docstring (`params.jl`), the `legalize`/projection
notes, and CLAUDE.md (architecture section + the point-aware-legalize note moves from
"deferred non-goal" to "implemented"). Add the guidance: set `point_padding =
marker_radius + small_gap`, or use the `markersize` attribute.

## Out of scope (deferred to a follow-up issue)

**Auto-snooping the sibling scatter's `markersize` at render time.** Investigated in
depth against Makie 0.24 source:

- `Makie.parent_scene(p)::Scene` reaches the axis scene; `scene.plots::Vector{Plot}`
  is iterable and could be filtered for `Scatter`; default `markersize = 9`
  (`theming.jl`), default `markerspace = :pixel` (`basic_plots.jl`), so a numeric
  `markersize` is the px diameter in the common case.
- **Disqualifying blocker:** `scene.plots` is a plain `Vector{Plot}` mutated by a bare
  `push!` (`scenes.jl`) — *not* an Observable. The recommended (and hero-example)
  layering draws `scatter!` *after* `textrepel!` so leaders tuck under markers; at the
  moment textrepel's lift first fires the sibling does not yet exist, and nothing
  retriggers the solve when it is later pushed. Making this work needs a
  defer-to-first-render + scene-poll redesign of the recipe's reactive model.
- Further problems: position→anchor matching ambiguity (multiple scatters, per-point
  `markersize` vectors), `markerspace = :data` and non-disc markers having no single
  radius, and private-API coupling (the same risk class the annotation hook already
  guards with a stability canary). It would also pull live scene state into the pure,
  deterministic solve layers.

Disproportionate to a clearance knob; filed as a separate opt-in enhancement.

## Affected files

- `src/solvers/projection.jl` — add the `n` anchor keep-out nodes to
  `legalize_and_drop`'s working arrays (the only logic change).
- `src/params.jl` — `point_padding` default `0.0 → 5.0`; reframe docstring.
- `src/recipe.jl` — new `markersize` attribute; derive `point_padding`; add to the
  solve `lift` dependencies; reframe `point_padding` docstring; one-time conflict warn.
- `src/annotation_algorithm.jl` — inherits the new `RepelParams` default; docstring
  touch if it names a `point_padding` default.
- `CLAUDE.md` — architecture + state notes (point-aware legalize implemented).
- `examples/readme_example.jl` — set `markersize = 9` (or explicit `point_padding`) so
  the hero example documents the new knob; regenerate `assets/example.png`.

## Testing

TDD. New and updated coverage:

1. **Clearance-floor regression** (`test_projection_solver.jl` or `test_legalize.jl`):
   on a dense fixture, after a full solve, assert `point_covered(anchor_j, unpadded_box_i,
   point_padding)` is `false` for every non-dropped label `i` and every anchor `j`
   (own and foreign) — i.e. the post-legalize floor holds, not just at side-select time.
2. **Hero-dataset `point_overlaps == 0`** (integration): the readme dataset with
   `markersize = 9` / `point_padding ≥ 5` yields `solve_stats(...).point_overlaps == 0`
   (currently 1).
3. **`markersize` attribute** (`test_integration.jl`): `markersize = m` ⇒ effective
   `point_padding = m/2 + 1`; both-set ⇒ `markersize` wins + one warning.
4. **Default-change fallout:** update numeric/golden assertions in the existing suites
   that shift under `point_padding = 5.0`. Re-run the full suite once, tee to an
   agent-scoped log under `test/output/`, grep — per CLAUDE.md.
5. Regenerate `assets/example.png` and eyeball: no marker-occluded labels.

## Acceptance (from the issue)

- A label's text box clears every marker disc (own and foreign) by at least the
  configured clearance **after** legalize — not just at side-select time. ✅ (§1)
- `point_overlaps == 0` on the README/hero dataset (currently 1). ✅ (test 2)
- Regenerated hero image shows no marker-occluded labels. ✅ (test 5)
- New regression test on a dense fixture asserting the post-legalize clearance floor.
  ✅ (test 1)
