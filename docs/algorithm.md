# How MakieTextRepel places labels

This document explains the placement algorithm behind `textrepel!` and
`TextRepelAlgorithm`, and how it compares to the two tools that inspired it,
[`ggrepel`](https://ggrepel.slowkow.com/) (R / ggplot2) and
[`adjustText`](https://adjusttext.readthedocs.io/) (Python / matplotlib). It covers
what the solver does, what it shares with those tools, and why it differs where it
does.

## The problem

Given `N` anchor points, each with a text label of known width and height, we choose
an offset for every label and draw a leader line back to its point. A good layout has
no label overlapping another, keeps labels clear of the markers, uses short leaders
that don't cross, and follows reading conventions.

This breaks into two parts:

- **Discrete** — which side of its anchor each label takes (upper-right, left,
  below, …), and which labels to drop when a region is too crowded to fit them all.
- **Continuous** — the exact pixel position of each label box once the side is chosen.

The discrete choice is the harder one. A label on the wrong side of its anchor can't
be fixed by nudging it a few pixels; it has to move to a different side. The solver
handles the two parts in separate stages for this reason.

## The pipeline

The default engine is the **`ProjectionSolver`**. It runs as a sequence of
deterministic stages in pixel space:

1. **Measure.** Each label's box is sized from its rendered extent using
   [TextMeasure.jl](https://github.com/jowch/TextMeasure.jl), with no `Scene`
   allocated. Plain strings, LaTeX, and Makie rich text are all supported. Because
   placement uses real glyph metrics rather than character-count estimates, every
   overlap and clearance test runs against the actual text box.
   Measurement and solving are separate compute nodes: text measurement is keyed on
   `text`/`fontsize`/`font` only, so position-only updates (animations) reuse measurements
   (issue #25).

2. **Seed (Voronoi + Imhof).** Each label gets a starting side. We compute a Voronoi
   cell per anchor and place the label at its most-preferred slot under the Imhof
   labelling convention (a cartographic priority order: upper-right first, then right,
   top, and so on) that fits inside that anchor's cell, falling back to upper-right
   when none fit. This spreads the initial guess out instead of starting every label
   in the same corner.

3. **Side-select.** A greedy pass re-picks each label's slot to minimize a ranked
   objective: first the number of overlaps (label–label, label–obstacle, and
   label-covering-a-marker, counted equally), then leader length, then slot
   preference. The ranking is strict — a shorter leader is never taken at the cost of
   an extra overlap, and a more-preferred slot is never taken at the cost of a longer
   leader.

4. **Crossing repair.** A 2-opt pass swaps pairs of label positions to remove
   leader-line crossings, with an iteration cap as a backstop.

5. **Legalize.** A projection pass removes any remaining box overlaps. For each
   overlapping pair it adds a one-dimensional separation constraint on the axis with
   the smaller overlap, then projects the boxes onto all the constraints at once
   (using Dykstra's algorithm), moving each box the least distance that clears it.
   Every movable box is kept inside the axis viewport, and each marker acts as a
   keep-out region so labels don't settle on a point. This is the stage that drives
   overlaps to zero (within a sub-pixel tolerance).

6. **Drop when over-capacity.** Some regions can't hold every label inside the
   viewport without overlap. When the legalize stage reports overlap it can't clear
   in-bounds, the most-overlapped label is dropped and the pass repeats, until the
   scene is clean or one label remains. Marker-clearance shortfalls don't trigger a
   drop — clearance is a soft constraint, so a tight marker gap never costs a label.

7. **Uncross.** A final swap-based search reduces leader crossings, accepting a swap
   only when the re-legalized layout is strictly better, and never one that
   re-introduces an overlap or drops a label.

A separate read-only cost function reports placement quality — overlap count, markers
covered, mean leader length, crossings. It is diagnostics only, exposed through
`solve_stats`, and never feeds back into placement.

## What this gives you

- **Deterministic output.** The default `ProjectionSolver` carries no RNG: its one
  randomized dependency (DelaunayTriangulation, for the Voronoi-informed seed) is
  seeded with `MersenneTwister(0)` over lexicographically sorted points, and every
  other stage is a pure deterministic algorithm. Identical inputs therefore produce
  byte-identical offsets and drop masks, so downstream consumers of the public
  `warm_solve` primitive may golden-test placement directly, and image-regression
  tests and reproducible publication figures come for free. (Exactly-coincident
  anchors fan out along a fixed golden-angle spiral — also fully reproducible.)
- **Zero overlap** (no two label boxes penetrating by more than about half a pixel) on
  any scene that fits in the viewport. This is the separation property of the
  projection at convergence, which it reaches well within the iteration caps that act
  as a backstop.
- **Markers stay clear.** Every anchor is a keep-out region, so a label never lands on
  its own point or a neighbour's. The clearance distance is the `point_padding` knob,
  or it can be derived from `markersize`.
- **In-bounds, or dropped.** Labels stay inside the axis. When a region is over
  capacity, labels are dropped rather than placed off-axis.

## Compared to `ggrepel` and `adjustText`

`ggrepel` and `adjustText` are the reference implementations of this idea, and
MakieTextRepel brings the same capability to Makie. It shares their goal and much of
their vocabulary, and differs mainly in how placement is computed.

### What's the same

- **The goal and the look.** Move overlapping labels away from their points and from
  each other, and draw a leader line back to each anchor.
- **The padding knobs.** `box_padding` and `point_padding` play the same roles as
  `ggrepel`'s `box.padding` and `point.padding`: breathing room around each label, and
  a clearance gap from the markers.
- **Real text measurement.** All three size labels from real text metrics —
  `ggrepel` via R's graphics device, `adjustText` via the matplotlib renderer,
  MakieTextRepel via TextMeasure.jl — not from character-count approximations.
- **Dropping when crowded.** Like `ggrepel`'s `max.overlaps`, labels are removed when
  a region can't hold them all, though the trigger here is a geometric over-capacity
  test rather than an overlap count.
- **Axis-constrained placement.** Labels stay within the plotting area.

### What's different, and why

- **Two phases instead of one force loop.** `ggrepel` and `adjustText` are continuous
  simulations: they assign repulsion forces between boxes and points and iterate until
  the layout settles or hits an iteration cap. MakieTextRepel splits the problem into a
  discrete side-selection phase and a separate overlap-removal phase. The reason is the
  wrong-side failure mode: a force loop never reconsiders which side of its anchor a
  label is on as a discrete decision, so a label that starts on a bad side tends to
  stay there with a long leader. Re-assigning the side is what side-selection does
  directly; it's the failure mode this phase is built to address.

- **A separation guarantee instead of best-effort.** A force loop stops where the
  forces happen to settle, with no guarantee that overlaps are gone, so some can remain
  on crowded scenes (both tools say as much — they report leftover overlaps at
  iteration cap-out). The legalize stage projects onto the non-overlap constraints, so
  on any scene that fits the result has no overlaps. The cost runs the other way:
  reaching zero overlap can leave a label slightly farther from its anchor than a force
  solver would. We take a clean layout as the better default.

- **Deterministic instead of randomized.** Both reference tools use randomness
  (`ggrepel` takes a `seed`; both perturb starting positions), so re-running can shift
  the labels. MakieTextRepel's placement is reproducible to the pixel. This matters for
  version-controlled figures and for testing.

- **Render-free measurement.** All three measure real text extents (see above); the
  difference is that MakieTextRepel does it through TextMeasure.jl without allocating a
  `Scene` or rendering, so measurement needs no backend round-trip.

### Why not a force loop?

A force-directed solver does ship in the package, behind the same solver interface, as
a non-default option. It isn't the default because the side-select → legalize pipeline
gives two things a force loop structurally can't: a zero-overlap guarantee and
deterministic output. The trade-off is the one noted above — reaching zero overlap can
push an individual label farther from its anchor. The force solver stays in the tree as
a comparison point and a fallback.

## Scope and non-goals

The solver is intentionally narrow:

- Obstacles are supplied explicitly as rectangles; it does not rasterize arbitrary
  scene content into a keep-out map.
- Leaders are straight lines, not bent or routed.
- The legalize stage uses a Dykstra projection in place of the scan-line VPSC
  algorithm it stands in for. VPSC would be faster on very large scenes and gives the
  same guarantee.

These keep the pipeline small and easy to follow.
