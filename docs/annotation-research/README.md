# `annotation!` deep research

Background research for the integration question: should MakieTextRepel.jl
plug into Makie's `annotation!` recipe via its `algorithm` hook (Option B),
remain a standalone recipe (Option A), or both (Option C)?

Three deep dives, each ~700–1200 lines with `file:line` citations against
Makie 0.24.10 (`~/.julia/packages/Makie/p9K7f`):

| File | Focus | Length |
|------|-------|--------|
| [`01-recipe-core.md`](./01-recipe-core.md) | signatures, attributes, compute graph, algorithm contract, NaN semantics, coordinate spaces | 1171 lines |
| [`02-path-style-arrow.md`](./02-path-style-arrow.md) | `Ann.Paths`, `Ann.Styles`, `Ann.Arrows`, BezierPath construction, connector emission logic, shrink/clip pipeline | 931 lines |
| [`03-limitations-and-stability.md`](./03-limitations-and-stability.md) | TODOs in the source, API stability, edge cases (rotation, log axes, N=500), interactivity gaps, `LabelRepel`'s shortcomings | 764 lines |

A runtime probe used by report 03 lives at
[`../../examples/probe_annotation_edges.jl`](../../examples/probe_annotation_edges.jl).

---

## Cross-cutting findings

These are the points that emerge by reading the three reports together. Each
links back to the report sections with the evidence.

### 1. We'd be coupling to internal API, not public API

Three hooks would matter for an integration; none are exported or
docstring-promised:

- **`Makie.calculate_best_offsets!`** — the algorithm dispatch. Marked in
  the recipe attribute docstring as "may change between non-breaking
  versions" ([01 §4](./01-recipe-core.md), [03 §2](./03-limitations-and-stability.md)).
- **`Ann.Paths` / `Ann.Styles` / `Ann.Arrows`** — `Ann` is `export`ed from
  `Makie.jl:435`, but the submodules and inner types aren't. Treat them as
  internal ([02 §11](./02-path-style-arrow.md)).
- **`__advance_optimization`** — the double-underscored compute input that
  drives the warm-start loop. Not exported; behavior conditions on "is this
  the only input that changed" ([01 §3](./01-recipe-core.md)).

### 2. The `reset::Bool` signature wart is real and load-bearing

The `::Automatic` overload takes a `reset::Bool` kwarg
(`annotation.jl:421-426`), but the `::LabelRepel` overload doesn't
(`annotation.jl:454-458`). The compute graph always passes `reset`, so:

- **Custom algorithms must accept `reset::Bool`** or they error out — exactly
  the bug we hit in the spike.
- `::Automatic` decides reset semantics for itself (line 439-441 fills with
  zero), then drops the kwarg before delegating. Custom algorithms should do
  the same.

This is undocumented; you find it by reading source. See
[03 §2](./03-limitations-and-stability.md) and [01 §4](./01-recipe-core.md).

### 3. The silent connector-skip at `annotation.jl:337`

`p2 in offset_bb && return` — if the data anchor lies inside the offset text
bbox, **no path is emitted at all**, period. No user knob disables it. This is
why our first spike PNG showed no visible connectors despite labels moving
50px: many of them were close enough that the anchor still fell inside the
(padded) text bbox.

This is the single biggest gotcha for any integration that wants visible
connectors comparable to ggrepel's. Workaround: push offsets harder so the
anchor falls fully outside, OR live with sparse connectors. See
[02 §5](./02-path-style-arrow.md).

### 4. `LabelRepel` itself is meaningfully weaker than what we have

Documented in [03 §3](./03-limitations-and-stability.md):

- Three knobs: `repel`, `attract`, `padding`. That's it.
- No convergence detection — always runs `maxiter` iterations.
- O(maxiter · N²) per solve; ~1.54s at N=500.
- No label dropping. No per-label parameters. No anisotropic forces.
  No axis-constrained motion. No deterministic init (one-shot centering
  bias, then pure dynamics).

Our stress tests already showed this empirically — LabelRepel collapses at
n=100 (labels exit viewport) and on co-located clusters. The source confirms
why.

### 5. `labelspace` is mostly cosmetic to the algorithm

The solver always operates in pixel space. Only `screenpoints_label` branches
on labelspace (annotation.jl:275-287). So a custom algorithm doesn't really
need to do anything different for `:relative_pixel` vs `:data` — they look
identical from the algorithm's perspective. See
[01 §6](./01-recipe-core.md).

### 6. No per-label "pin" or mixed manual/auto mode

TODO at `annotation.jl:447-449` says: "make it so some positions can be fixed
and others are not (NaNs)." Today the check is binary — either every
`textpositions_offset` is finite (manual mode, algorithm skipped) or any of
them is NaN (auto mode, algorithm runs on all of them).

This is also where our `max_overlaps` (label dropping) doesn't translate —
annotation has no concept of "this label exists in the input but should not
render."

### 7. The Paths/Styles surface is small and partly broken for extension

[02 §6](./02-path-style-arrow.md): exactly three Path types (`Line`,
`Corner`, `Arc`), three Style types (`Line`, `LineArrow`, `WithText`), two
Arrow types (`Line`, `Head`). The contract per Path is just two methods
(`startpoint`, `connection_path`), and per Style is one
(`annotation_style_plotspecs`).

Extending it is mechanically possible, but `shrink_path` and
`circle_intersection` (the clipping helpers) only support `LineTo` and
**circular** `EllipticalArc` — a path made of `CurveTo` commands would
`MethodError`. So the extension surface is real but rough.

### 8. No interactivity

[03 §5](./03-limitations-and-stability.md): no drag, no hover, no
manual-nudge-after-auto-placement. `p.offsets` is the only data-out
handoff. `advance_optimization!` and `__advance_optimization` are the only
step-control primitives.

---

## Net read for the integration question

Pre-research, the lean was C ("ship both: textrepel! recipe AND
TextRepelAlgorithm plug-in").

The research mostly confirms that, with two specific reinforcements:

1. **The plug-in side has more sharp edges than the spike suggested.** Three
   internal hooks, undocumented `reset` semantics, silent connector skip,
   stale `shrink_path` helpers — none individually fatal, but they add up to
   "we're on top of stuff that can change." For a v0.1 release, this is
   probably OK; long-term, it's a maintenance tax.

2. **The recipe side's value is real and grounded.** `LabelRepel` genuinely
   lacks dropping, anisotropy, only_move, convergence detection, and viewport
   clamping. These aren't decorative — the stress tests showed they're the
   difference between legible and illegible plots at n=100. Our recipe
   exposes them as flat kwargs and they compose naturally with the box
   rendering and connector suppression.

Concrete recommendation for next session: ship `textrepel!` as the primary
surface. Ship `TextRepelAlgorithm` as an optional companion in the same
package, with an explicit note in its docstring that it depends on Makie
internals and may need updates when Makie does. Both share the solver, so
the marginal cost of the second surface is just the ~80-line wrapper.
