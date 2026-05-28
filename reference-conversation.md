<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" style="height:64px;margin-right:32px"/>

# Is there an equivalent package for ggrepel or adjustText for julia?

There is **no direct, dedicated Julia equivalent** of `ggrepel`/`adjustText` — it's a genuine gap in the Julia plotting ecosystem. Here's the current state:

## Gadfly.jl Has It Built-In

The closest native solution is **`Gadfly.jl`'s `Geom.label`**, which has label repulsion built directly into its geometry layer. It automatically optimizes label positions to avoid overlaps and hides irresolvable ones. The downside: it hides labels rather than always finding a placement, and Gadfly has fallen out of favor compared to Makie for most use cases.[^1_1]

## Makie.jl — Manual or Roll Your Own

Makie has no `text_repel` recipe out of the box. A GitHub issue on GraphMakie.jl documents an attempt to implement a `text_repel!` recipe by porting R code, but it was never released as a standalone package. The StackOverflow thread on CairoMakie label placement describes the force-directed algorithm (attractive force to anchor label to point, repulsive forces between labels and away from other points) but leaves the implementation to the user.[^1_2][^1_3]

## Workarounds Available

- **`RCall.jl`** — call R's `ggrepel` directly from Julia. This is actually a real pattern used in the Julia community for this exact use case. Verbose, but gives you the full ggrepel behavior.[^1_4]
- **`PyCall.jl`** — call Python's `adjustText` via Julia's Python interop.
- **`TidierPlots.jl`** — a ggplot2 reimplementation in Julia (announced at JuliaCon 2024), but it does not currently include a `geom_text_repel` equivalent.[^1_5]


## Bottom Line

If you're on Makie (which fits your stack), the honest answer is you'd need to implement a simple force-directed label nudger yourself, or fall back to `RCall`/`PyCall`. Given your Julia/Makie usage, a minimal custom `text_repel!` recipe for CairoMakie would be reasonably straightforward — the algorithm is well-defined and a few dozen lines. Would it be worth me sketching that out as a Makie recipe?
<span style="display:none">[^1_10][^1_11][^1_12][^1_13][^1_14][^1_15][^1_16][^1_17][^1_18][^1_19][^1_20][^1_21][^1_22][^1_23][^1_24][^1_25][^1_26][^1_27][^1_28][^1_29][^1_30][^1_31][^1_32][^1_33][^1_34][^1_35][^1_36][^1_37][^1_38][^1_39][^1_40][^1_6][^1_7][^1_8][^1_9]</span>

<div align="center">⁂</div>

[^1_1]: http://gadflyjl.org/v0.7/lib/geoms/geom_label.html

[^1_2]: https://stackoverflow.com/questions/73976927/makie-non-overlapping-label-placement-algorithm-for-scatter-plots

[^1_3]: https://github.com/MakieOrg/GraphMakie.jl/issues/131

[^1_4]: https://discourse.julialang.org/t/how-to-make-this-plot-in-julia/75065

[^1_5]: https://www.youtube.com/watch?v=33yik1ciUWE

[^1_6]: https://pypi.org/project/adjustText/

[^1_7]: https://adjusttext.readthedocs.io

[^1_8]: https://anaconda.org/conda-forge/adjustText

[^1_9]: https://github.com/has2k1/plotnine/issues/218

[^1_10]: https://stackoverflow.com/questions/57701052/how-do-i-use-adjust-text-with-plotnine

[^1_11]: https://www.linkedin.com/posts/jia-geng-chang_adjusttext-is-a-nice-library-in-python-that-activity-7323187405954449409-i5kO

[^1_12]: https://dokk.org/documentation/matplotlib-adjusttext/v1.0/

[^1_13]: https://stackoverflow.com/questions/34693991/repel-annotations-in-matplotlib

[^1_14]: https://avt.im/blog/archive/r-packages-ggplot-in-julia/

[^1_15]: https://www.rdocumentation.org/packages/ggrepel/versions/0.8.2

[^1_16]: https://python-graph-gallery.com/web-text-repel-with-matplotlib/

[^1_17]: https://www.r4photobiology.info/galleries/nudge-and-repel.html

[^1_18]: https://ggrepel.slowkow.com

[^1_19]: https://www.youtube.com/watch?v=PerqyXWdRJQ

[^1_20]: https://discourse.julialang.org/t/how-to-make-a-plot-with-multiple-titles-and-labels-in-makie/89137

[^1_21]: https://docs.makie.org/dev/api

[^1_22]: https://jtr13.github.io/cc20/some-techniques-for-label-setting.html

[^1_23]: https://www.youtube.com/watch?v=SLkJyrqfHg4

[^1_24]: https://ggrepel.slowkow.com/articles/examples

[^1_25]: https://apxml.com/courses/julia-for-machine-learning/chapter-2-julia-data-manipulation-preparation/julia-data-visualization

[^1_26]: https://github.com/slowkow/ggrepel/issues/89

[^1_27]: https://sefffal.github.io/PairPlots.jl/dev/guide

[^1_28]: https://r-graph-gallery.com/package/ggrepel.html

[^1_29]: https://www.reddit.com/r/Julia/comments/1ol9zr6/do_you_prefer_plotsjl_or_makiejl_or_other_plots/

[^1_30]: https://cran.r-project.org/package=ggrepel/vignettes/ggrepel.html

[^1_31]: https://plotly.com/julia/text-and-annotations/

[^1_32]: https://discourse.julialang.org/t/placing-a-label-at-an-arbitrary-place-using-gadfly/66528

[^1_33]: https://www.youtube.com/watch?v=qsUb2KjJ7tk

[^1_34]: https://rdrr.io/cran/ggrepel/man/geom_text_repel.html

[^1_35]: https://blog.devgenius.io/the-most-underrated-feature-of-the-julia-programming-language-the-package-manager-652065f45a3a

[^1_36]: https://www.rdocumentation.org/packages/ggrepel/versions/0.5/topics/geom_label_repel

[^1_37]: https://aviatesk.github.io/JET.jl/dev/

[^1_38]: https://www.youtube.com/watch?v=QnctM0MUox8

[^1_39]: https://stackoverflow.com/questions/48692705/text-repel-with-a-position-argument-in-ggplot-r

[^1_40]: https://www.youtube.com/watch?v=KldNhe4kvuk


---

# I would like to make one. Can you help me do the background research on ggrepel, textAdjust, and the GraphMakie.jl attempt? Help me understand how these packages actually work (I have some guesses but I want to ground them in facts).

Here's a thorough breakdown of how each approach actually works, grounded in source code and documentation.

***

## ggrepel: The Reference Implementation

ggrepel's core loop is implemented in **C++ (via Rcpp)** and called as `repel_boxes2()` from R. The R layer handles coordinate transformation and bounding box measurement; the C++ layer does the actual physics simulation. Here's what the algorithm does:[^2_1]

### Bounding Box Measurement (R side)

Before any physics, ggrepel measures the **actual rendered pixel dimensions** of each text label using grid's `textGrob`, `grobX`, and `grobY`. These are real typographic extents — not estimates — padded by `box.padding` (default 0.25 lines). This is critical: the boxes are computed in *native plot coordinates* (0–1 normalized), which is why labels auto-reposition on plot resize.[^2_1]

### Force-Directed Physics Loop (C++ side)

Each iteration applies three forces to every label:[^2_2]

1. **Repulsion from other labels** (`force` parameter, default 1, scaled to `1e-6`): proportional to overlap area between bounding boxes, directed along the vector between box centers
2. **Repulsion from data points** (same `force`): labels repel from their anchor point and all other data points within range, modeled as circles with `point.size + point.padding` radius
3. **Attraction to anchor** (`force_pull`, default 1, scaled to `1e-2`): a weak spring pulling each label back toward the data point it labels

The loop runs until either `max.time` (0.5s default) or `max.iter` (10,000) is exhausted, whichever comes first. The simulation is non-deterministic unless you set `seed`.[^2_2]

### Overflow Handling

Labels that overlap more than `max.overlaps` (default 10) other objects are **dropped entirely** rather than placed poorly. This is a deliberate design choice — ggrepel prioritizes legibility over completeness.[^2_3]

### Connector Lines

After placement, ggrepel computes the **intersection of the line from the data point to the label centroid with the label rectangle's edges**, then draws a segment/curve only if the distance exceeds `min.segment.length` (0.5 lines). The connection point is geometrically exact, not heuristic.[^2_1]

***

## adjustText: The Python Port

adjustText is explicitly inspired by ggrepel but takes a **different algorithmic approach**. Where ggrepel integrates repulsion and attraction simultaneously in a physics loop, adjustText uses a **two-phase strategy**:[^2_4][^2_5]

### Phase 1: Explosion

Before iterating, it does a forced **"explosion" pass** — it moves every text away from nearby texts and static objects using `force_explode`. The `explode_radius` (default: `"auto"`, based on mean text size) defines how far to look for neighbors. This pre-separates perfectly overlapping labels (which would have zero-length repulsion vectors and never move otherwise).

### Phase 2: Iterative Force Loop

Each iteration applies four distinct force categories:[^2_4]


| Force | Parameter | Default | Effect |
| :-- | :-- | :-- | :-- |
| `force_text` | `(0.1, 0.2)` | Separate x/y | Repulsion between text bounding boxes |
| `force_static` | `(0.1, 0.2)` | Separate x/y | Repulsion from data points and `objects` |
| `force_pull` | `(0.01, 0.01)` | Separate x/y | Attraction back to original anchor |
| `force_explode` | `(0.1, 0.5)` | Separate x/y | Used in phase 1 and for random shifts |

The key difference from ggrepel: forces are **tuples with separate x and y magnitudes**, giving you anisotropic control (e.g., constrain movement to one axis). The `only_move` dict lets you restrict specific force types to only `x`, `y`, `x+`, `x-` etc.[^2_4]

### Key Implementation Differences

- adjustText operates in **display coordinates** (pixels), not normalized data units. `max_move` defaults to `(10, 10)` pixels per iteration, capping how far a label moves each step.
- `pull_threshold` (default 10px): if a label is already within 10px of its target along an axis, the pull force is suppressed along that axis — preventing oscillation near the anchor.
- Unlike ggrepel, adjustText **never hides labels**. It always finds a placement, even if crowded.
- The `prevent_crossings` option (experimental) adds a post-processing pass to untangle connector arrow crossings.[^2_4]

***

## GraphMakie.jl: The Incomplete Julia Attempt

The GraphMakie issue \#131 attempted a `text_repel!` recipe for Makie, but it was never completed or packaged. What the issue documents is essentially a **direct port of ggrepel's R logic into Julia**, using:[^2_6]

- `NetworkLayout.jl`'s spring layout as inspiration for the force formulation
- Makie's `text!` recipe as the rendering primitive
- Manual bounding box calculation using Makie's `boundingbox` on text objects

The fundamental problem that stalled it: **Makie does not expose text bounding boxes synchronously during the draw pipeline** the way ggplot2's grid system does. In ggrepel, `makeContent.textrepeltree` fires *during rendering* when grid has already measured the glyphs. In Makie, you'd need to either (a) pre-estimate bbox from font metrics, or (b) force a render pass first and inspect results — neither of which fits cleanly into Makie's `@recipe` system.[^2_7]

***

## What This Means for Your Implementation

The architecture decision you'll face first:

**Option A (ggrepel-style):** Measure real text bboxes in a `makeContent`-equivalent hook (Makie's `Observables`), then run a C/Julia physics kernel. Accurate but requires hooking into Makie's scene update lifecycle.

**Option B (adjustText-style):** Estimate bboxes from font metrics upfront (simpler, slightly wrong for variable-width fonts), run the iterative loop in pure Julia, render. Much easier to prototype.

The ggrepel approach is more correct but harder to integrate; adjustText's approach is actually a better fit for a first Julia recipe because you control the entire pipeline. The anisotropic force tuning in adjustText is also genuinely more useful than ggrepel's single `force` scalar, and the two-phase explosion + iterate structure converges faster in practice.

One blind spot to think about: **coordinate system normalization**. Both ggrepel and adjustText operate in display/normalized coordinates internally, not data coordinates. Your recipe will need to convert from data space → pixel space (or normalized Makie coordinates) before running physics, then convert back — otherwise the aspect ratio of the plot will distort your force vectors asymmetrically.
<span style="display:none">[^2_10][^2_11][^2_12][^2_13][^2_14][^2_15][^2_16][^2_17][^2_18][^2_19][^2_20][^2_21][^2_22][^2_23][^2_24][^2_25][^2_26][^2_27][^2_8][^2_9]</span>

<div align="center">⁂</div>

[^2_1]: https://rdrr.io/cran/ggrepel/src/R/geom-text-repel.R

[^2_2]: https://ggrepel.slowkow.com/articles/examples

[^2_3]: https://ggrepel.slowkow.com/news/

[^2_4]: https://adjusttext.readthedocs.io

[^2_5]: https://github.com/Phlya/adjustText

[^2_6]: https://github.com/MakieOrg/GraphMakie.jl/issues/131

[^2_7]: https://graph.makie.org/stable/generated/plots/

[^2_8]: https://gist.github.com/MoritzStefaner/1377729

[^2_9]: https://www.r4photobiology.info/galleries/nudge-and-repel.html

[^2_10]: https://ggrepel.slowkow.com/articles/related-work

[^2_11]: https://g6.antv.antgroup.com/en/manual/layout/force-layout

[^2_12]: https://pmc.ncbi.nlm.nih.gov/articles/PMC12306815/

[^2_13]: https://www.youtube.com/watch?v=Cbaz95n2-fk

[^2_14]: https://stackoverflow.com/questions/75175076/graphmakie-jl-cuts-off-text

[^2_15]: https://www.yworks.com/pages/force-directed-graph-layout

[^2_16]: https://web.tohoku.ac.jp/kc_kyomu/computer_seminar1/py/textbook_en/spring.html

[^2_17]: https://juliapackages.com/p/graphmakie

[^2_18]: https://www.youtube.com/watch?v=PTBuq0CXpWs

[^2_19]: https://github.com/dantrag/spring_simulator

[^2_20]: https://github.com/slowkow/ggrepel/issues/202

[^2_21]: https://r-graph-gallery.com/package/ggrepel.html

[^2_22]: https://dokk.org/documentation/matplotlib-adjusttext/v1.0/

[^2_23]: https://ggrepel.slowkow.com

[^2_24]: https://github.com/Phlya/adjustText/wiki

[^2_25]: https://www.youtube.com/watch?v=qsUb2KjJ7tk

[^2_26]: https://github.com/Phlya/adjustText/issues/91

[^2_27]: https://stackoverflow.com/questions/79780403/ggrepel-still-overlapping-text


---

# Yeah, this tracks. Ok, the next step is to investigate whether we can calculate element bounding boxes in Makie by potentially hijacking its internals.

This is a nuanced area — there are several distinct strategies, each with real tradeoffs. Here's what the sources tell us.

***

## The Core Problem

Makie's `boundingbox` on a `text!` plot object only gives you a box over the **anchor points**, not the actual glyph extents — *when `markerspace = :pixel` (the default)*. The docs are explicit: "its extent depends on the current projection of the axis it is in". This is the fundamental pain point.[^3_1]

## Strategy 1: `boundingbox()` After Plotting (Post-Hoc)

The working approach documented in issue \#2424 is: **plot first, then query**. After a `text!` call, you can get real pixel-space bounding boxes via:[^3_2]

```julia
lbl = text!(ax, "Label", position, align=(:left, :bottom))
bb = map(ax.scene.camera.pixel_space, ax.scene.camera.projectionview, lbl) do _, _, lbl
    boundingbox(lbl)
end
```

The `map` over `pixel_space` and `projectionview` Observables ensures the bbox recomputes when the camera changes. The result is in **pixel coordinates**. You then shift the label by mutating its `position` Observable, or use `space = :pixel` for final placement.[^3_2]

**Verdict for text_repel:** This is "plot, measure, shift" — doable but inelegant. You'd render labels in a first pass, run your physics loop with pixel-space boxes, then update positions. The risk: triggering layout recomputation on every position update during the solver loop could be slow.

## Strategy 2: `string_boundingbox` Pre-Render (Internal API)

Issue \#3860 documents that Makie *does* have `string_boundingbox()` that operates on a `GlyphCollection` without needing a rendered plot. The pipeline is:[^3_3]

```julia
# Internal, not public API
gc = Makie.layout_text(str, fontsize, font, align, rotation, justification, lineheight)
bb = Makie.string_boundingbox(gc)  # returns Rect2f in font units
```

The catch, confirmed by Makie maintainer @jkrumbiegel: *"The functions to do the boundingbox calculations are all there, but they operate on `GlyphCollection` so you first need to pass your text with all its visual attributes that affect boundingboxes through the layouting algorithm."* So you need to fully specify `font`, `fontsize`, `rotation`, `justification`, `lineheight` — everything that affects glyph layout.[^3_2]

The returned bbox is in **font-metric units** (not yet in screen pixels), so you still need `Makie.project` or `Makie.scene_to_screen` to convert to pixel space.

**Verdict for text_repel:** This is the "right" path for a pre-layout solver. You can compute all bboxes upfront, run the full physics loop, and then call `text!` once with final positions. The downside: `layout_text` and `string_boundingbox` are **private, unstable API** — they've changed between versions and will change again.

## Strategy 3: `markerspace = :data`

Setting `markerspace = :data` makes `boundingbox` include actual glyph extents in data coordinates. This sidesteps pixel-space entirely. But it means your force vectors must also be in data space, which reintroduces the **aspect ratio distortion problem** — if your data axes aren't square, forces computed in data coordinates are geometrically wrong (a 1-unit displacement horizontally ≠ 1-unit vertically in screen space).[^3_1]

**Verdict for text_repel:** Only works correctly if you normalize to screen space before computing repulsion, defeating the purpose.

## Strategy 4: `textlabel` (New in Recent Makie)

Makie now has a `textlabel` plot type that internally manages text + background shape, using the text bounding box to scale a shape. Its source computes bboxes in the same `GlyphCollection` pathway as strategy 2. You can potentially read its internal bbox computation as a model, but it doesn't expose the bbox directly.[^3_4]

***

## Recommended Architecture for Your Recipe

Based on all of this, the cleanest approach is a **two-Observable pipeline**:

1. **Measure phase**: Use `layout_text` + `string_boundingbox` (internal API) to compute all label extents in font units, then `Makie.project` to convert to pixel space. Pin this behind a version check.
2. **Solve phase**: Run your physics loop entirely in pixel space. This isolates the solver from Makie internals.
3. **Render phase**: Call `text!` once with final positions using `space = :pixel` for placement or translate back to data coordinates via the inverse transform.
4. **Reactive updates**: Wrap the measure+solve+render chain in an `on(ax.scene.camera.pixel_space)` Observable so it recomputes on zoom/resize.

The biggest practical risk: `layout_text`'s signature has shifted across Makie versions (it's not public API). You'll want to version-gate it or write a small compatibility shim. The \#4729 issue even notes that `string_boundingbox` can return NaN under degenerate font conditions, so defensive checks are warranted.[^3_5]
<span style="display:none">[^3_10][^3_11][^3_12][^3_13][^3_14][^3_15][^3_16][^3_17][^3_18][^3_19][^3_20][^3_21][^3_22][^3_23][^3_24][^3_25][^3_26][^3_27][^3_28][^3_29][^3_30][^3_31][^3_32][^3_33][^3_34][^3_6][^3_7][^3_8][^3_9]</span>

<div align="center">⁂</div>

[^3_1]: https://docs.makie.org/dev/reference/plots/text

[^3_2]: https://github.com/MakieOrg/Makie.jl/issues/2424

[^3_3]: https://github.com/MakieOrg/Makie.jl/issues/3860

[^3_4]: https://docs.makie.org/dev/reference/plots/textlabel

[^3_5]: https://github.com/MakieOrg/Makie.jl/issues/4729

[^3_6]: https://discourse.julialang.org/t/fill-a-bounding-box-behind-text-in-makie/101502

[^3_7]: https://stackoverflow.com/questions/11526627/bounding-box-of-n-glyphs-given-individual-bboxes-and-advances

[^3_8]: https://docs.makie.org/dev/api

[^3_9]: https://docs.makie.org/v0.22/how-to/match-figure-size-font-sizes-and-dpi

[^3_10]: https://freetype.org/freetype2/docs/glyphs/glyphs-3.html

[^3_11]: https://www.youtube.com/watch?v=VT1XY1-fNlY

[^3_12]: https://juliapackages.com/p/tueplots

[^3_13]: https://developer.apple.com/documentation/appkit/nsatstypesetter/boundingbox(forcontrolglyphat:for:proposedlinefragment:glyphposition:characterindex:)

[^3_14]: https://discourse.julialang.org/t/cairomakie-text-box/103783

[^3_15]: https://docs.makie.org/dev/explanations/fonts

[^3_16]: https://forum.processing.org/two/discussion/19869/is-it-really-not-possible-to-determine-the-precise-height-of-a-text.html

[^3_17]: https://github.com/JuliaPlots/Makie.jl/issues/972

[^3_18]: https://discourse.julialang.org/t/fill-a-bounding-box-of-a-text-in-makie/60640

[^3_19]: https://docs.makie.org/dev/explanations/blocks

[^3_20]: https://docs.makie.org/dev/explanations/faq

[^3_21]: https://docs.makie.org/dev/explanations/specapi

[^3_22]: https://www.filibeto.org/unix/tru64/lib/ossc/doc/freetype2-2.0.1/docs/tutorial/step2.html

[^3_23]: https://bkamins.github.io/julialang/2023/12/01/plot.html

[^3_24]: https://github.com/JuliaPlots/Makie.jl/issues/354

[^3_25]: https://discourse.julialang.org/t/migrating-from-glmakie-0-4-7-to-current/79913

[^3_26]: https://github.com/MakieOrg/Makie.jl/discussions/4214

[^3_27]: https://discourse.julialang.org/t/code-listing-in-makie-jl-figure/74395

[^3_28]: https://github.com/JuliaPlots/Makie.jl/issues/183

[^3_29]: https://github.com/MakieOrg/Makie.jl/issues/3105

[^3_30]: https://github.com/MakieOrg/Makie.jl

[^3_31]: https://github.com/MakieOrg/Makie.jl/issues/50

[^3_32]: https://docs.makie.org/stable/tutorials/layout-tutorial/

[^3_33]: https://github.com/MakieOrg/Makie.jl/discussions/1906

[^3_34]: https://docs.makie.org/v0.22/reference/plots/text


---

# Tell me about textlabel

`textlabel` is a relatively new, purpose-built Makie plot type that renders text with a fitted background shape. Here's everything relevant to your use case:

## What It Does

`textlabel` plots text at given positions with a **background shape that automatically rescales to fit the text's bounding box**. This is the key detail: the background shape transformation pipeline internally computes the true glyph bounding box — meaning Makie *already has* the bbox computation you need, and `textlabel` is where it's exposed.[^4_1]

The rendering pipeline:

1. Text is laid out via the same `GlyphCollection` path
2. The padded text bbox is computed
3. A user-supplied `shape` (any `GeometryPrimitive`, `BezierPath`, or `(origin, size) -> points` function) is **transformed to fit the bbox** using `shape_limits` as the reference frame

## Key Attributes Relevant to text_repel

| Attribute | Default | Notes |
| :-- | :-- | :-- |
| `offset` | `(0.0, 0.0)` | Shifts label from anchor in `markerspace` units — this is your primary nudge vector |
| `padding` | `4` | Pixels of space between glyph edges and shape boundary |
| `markerspace` | `:pixel` | Space in which offset and shape sizing live — pixel-space by default |
| `space` | `:data` | Space for the anchor position itself |
| `text_align` | `(:center, :center)` | Text alignment within the box |
| `keep_aspect` | `false` | Whether to preserve shape aspect ratio when rescaling to text bbox |
| `shape` | `Rect2f(0,0,1,1)` | Background shape; `(0,0)` = lower-left corner of padded bbox, `(1,1)` = upper-right |

The `shape` as a **function** `(origin, size) -> coordinates` is particularly interesting  — it means the background is computed lazily from the bbox dimensions, which implies the bbox is available as a concrete value at draw time.[^4_1]

## Why This Matters for Your Recipe

`textlabel` is architecturally your **best entry point**, not just a user-facing API. Two concrete reasons:

**1. The bbox is already computed.** The `shape = (origin, size) -> ...` callback receives `origin::Point2f` and `size::Vec2f` of the padded text bbox in pixel space. If you hijack this: you can register a shape function that **captures the bbox into an external buffer** as a side effect, giving you ground-truth pixel dimensions without calling any private API. This is a legitimate, stable-API trick:

```julia
bboxes = Dict{Int, Rect2f}()
function spy_shape(i)
    return function(origin, size)
        bboxes[i] = Rect2f(origin, size)
        return [origin, origin + Point2f(size[^4_1], 0),
                origin + Point2f(size[^4_1], size[^4_2]), origin + Point2f(0, size[^4_2])]
    end
end

# First pass: render with spy shapes to capture bboxes
for (i, (pos, str)) in enumerate(zip(positions, labels))
    textlabel!(ax, pos; text=str, shape=spy_shape(i))
end
# Now bboxes[i] is populated after the first draw
```

**2. `offset` is in pixel space.** After running your physics solver, you set `plot.offset[] = new_offset_px` to move the label. No coordinate system transformation needed — the solver works in pixels, the output goes directly into `offset`. This is cleaner than the `space = :pixel` position approach from the previous discussion.

## Caveats

- `markerspace = :pixel` (default) means the **shape and offset don't zoom/scale with data axes** — which is actually correct behavior for a text repel recipe. Labels should stay the same pixel size when you zoom. If you want labels to scale with data (e.g., for GISAXS annotation maps), you'd set `markerspace = :data`, but then your physics loop must work in data coordinates and forces become anisotropic.[^4_2]
- `textlabel` still has some rough edges — `Cycled()` for `text_color` was broken as of November 2025, so it's not fully production-hardened.[^4_3]
- The spy-shape bbox capture relies on the shape callback being called **synchronously during the first render** — true for CairoMakie (immediate rendering), but for GLMakie/WGLMakie you'd need to wait on the render Observable or use `display()` first before reading the captured values.

The shape-callback spy is probably the cleanest stable-API path for your pre-solver measurement, and `offset` is the cleanest stable-API path for post-solver placement. It lets you avoid touching `layout_text` / `string_boundingbox` entirely.
<span style="display:none">[^4_10][^4_11][^4_12][^4_13][^4_14][^4_15][^4_4][^4_5][^4_6][^4_7][^4_8][^4_9]</span>

<div align="center">⁂</div>

[^4_1]: https://docs.makie.org/dev/reference/plots/textlabel

[^4_2]: https://discourse.julialang.org/t/keeping-textlabel-s-at-a-fixed-size-in-makie/130435

[^4_3]: https://github.com/MakieOrg/Makie.jl/issues/5395

[^4_4]: https://docs.makie.org/dev/reference/plots/text

[^4_5]: https://docs.makie.org

[^4_6]: https://tlienart.github.io/Makie.jl/dev/examples/layoutables/label/

[^4_7]: https://discourse.julialang.org/t/how-to-set-xlabel-and-ylabel-in-makie-recipe/68279

[^4_8]: https://tlienart.github.io/Makie.jl/dev/

[^4_9]: https://discourse.julialang.org/t/text-display-or-indicator-in-makie-jl/73957

[^4_10]: https://github.com/MakieOrg/Makie.jl/issues/2424

[^4_11]: https://discourse.julialang.org/t/cairomakie-text-box/103783

[^4_12]: https://docs.makie.org/dev/api

[^4_13]: https://www.youtube.com/watch?v=SLkJyrqfHg4

[^4_14]: https://github.com/MakieOrg/Makie.jl/issues/5171

[^4_15]: https://discourse.julialang.org/t/makie-text-with-variable-styling/81666


---

# Ok, let's sketch out the outline of what this would look like? Is there a way to insert this operation into the declarative figure spec so that it runs on render?

Good, this gives a clear picture. Here's the full architectural sketch.

***

## The Core Lifecycle Problem

Makie's `@recipe` system is **declarative and reactive**, not imperative. The `plot!` method runs *once* at construction time to wire up Observables; it does not run on every draw. This means you can't do "measure, solve, re-render" in a single synchronous pass inside `plot!`. The key insight from the docs: *"The plotting calls in your recipe should only be done once, not on every observable update. But the input observables to these inner plotting calls should be lifted from the recipe input observables."*[^5_1][^5_2]

This is both a constraint and a gift — if you model the solver as an Observable transformation, it updates reactively on zoom/resize for free.

***

## Outline: `textrepel!` Recipe

### Structure

The recipe wraps two things: a `textlabel!` call (for drawing) and a `lines!`/`linesegments!` call (for connector lines). The positions fed to `textlabel!` are **lifted from a solver Observable**, not from the raw input positions.

```julia
@recipe(TextRepel, positions, labels) do scene
    Attributes(
        # Physics
        force_text    = 0.1,
        force_static  = 0.1,
        force_pull    = 0.01,
        max_iter      = 200,
        padding       = 4.0,       # px, fed to textlabel
        # Connector lines
        draw_lines    = true,
        min_dist      = 5.0,       # px — suppress line if label barely moved
        linecolor     = :gray60,
        # Passthrough to textlabel
        fontsize      = 12,
        color         = :black,
    )
end
```


### The `plot!` Body

The tricky part is the **two-pass bootstrap problem**. Here's the approach that avoids private API:

```julia
function Makie.plot!(p::TextRepel)

    # 1. Bbox capture store — filled by shape spy on first render
    bboxes = Observable(Dict{Int, Rect2f}())

    # 2. Build spy shapes that populate bboxes as a side effect
    spy_shapes = @lift begin
        n = length($(p.labels))
        [function(origin, size)
            $(bboxes)[][i] = Rect2f(origin, size)   # capture bbox
            notify($(bboxes))
            # Return a transparent rect (no visible background in first pass)
            [origin, origin .+ Point2f(size[^5_1],0),
             origin .+ Point2f(size[^5_1],size[^5_2]), origin .+ Point2f(0,size[^5_2])]
        end for i in 1:n]
    end

    # 3. First-pass textlabel — invisible, just to trigger bbox capture
    # Uses raw positions, no offset yet
    ghost = textlabel!(p, p.positions, text=p.labels,
                       shape=spy_shapes, color=(:black, 0),
                       markerspace=:pixel, visible=false)

    # 4. Solver: lifts on bboxes + camera pixel_space
    #    Runs whenever bboxes populate OR camera changes (zoom/resize)
    solved_offsets = @lift begin
        bb = $(bboxes)
        isempty(bb) && return zeros(Point2f, length($(p.labels)))

        # Convert anchor positions to pixel space
        px_positions = Makie.project.(
            Ref($(p).parent.scene),   # or ax.scene
            $(p.positions)
        )

        # Run physics solver (pure Julia, pixel space)
        _solve_repel(px_positions, bb,
                     $(p.force_text), $(p.force_static),
                     $(p.force_pull), $(p.max_iter))
        # Returns Vec2f[] of pixel-space offsets from original anchor
    end

    # 5. Final visible textlabel — real background, solved offsets
    textlabel!(p, p.positions,
               text=p.labels,
               offset=solved_offsets,
               markerspace=:pixel,
               padding=p.padding,
               fontsize=p.fontsize,
               color=p.color)

    # 6. Connector line segments
    # Only draw when offset > min_dist
    segments = @lift begin
        offsets = $(solved_offsets)
        pos     = $(p.positions)
        thresh  = $(p.min_dist)
        # Interleave: [anchor₁, labelcenter₁, anchor₂, labelcenter₂, ...]
        # filtered by norm(offset) > thresh
        _build_segments(pos, offsets, thresh)
    end

    linesegments!(p, segments,
                  color=p.linecolor,
                  visible=p.draw_lines)

    return p
end
```


### The Solver (`_solve_repel`)

Pure Julia, no Makie dependency — easy to test in isolation:

```julia
function _solve_repel(anchors::Vector{Point2f},
                      bboxes::Dict{Int,Rect2f},
                      force_text, force_static, force_pull,
                      max_iter)
    n = length(anchors)
    offsets = zeros(Vec2f, n)

    for _ in 1:max_iter
        Δ = zeros(Vec2f, n)
        for i in 1:n
            bi = translate(bboxes[i], anchors[i] + offsets[i])

            # Repel from other labels
            for j in 1:n
                i == j && continue
                bj = translate(bboxes[j], anchors[j] + offsets[j])
                overlap = _overlap_vec(bi, bj)
                Δ[i] += force_text * overlap
            end

            # Repel from anchor points (all of them, not just own)
            for j in 1:n
                d = (anchors[i] + offsets[i]) - anchors[j]
                nd = norm(d)
                nd < 1f-6 && continue
                Δ[i] += force_static * d / nd^2
            end

            # Attract to own anchor
            Δ[i] -= force_pull * offsets[i]
        end
        offsets .+= Δ
    end
    return offsets
end
```


***

## Does SpecApi / Declarative Figure Work?

Yes, with a caveat. SpecApi (`Makie.SpecApi`) introduced in v0.20 lets you describe plots as specs (`PlotSpec`) that Makie diffs and updates. You can do:[^5_3]

```julia
# Declarative-style
fig = Figure()
obs_data = Observable((positions, labels))
spec = @lift Makie.PlotSpec(:TextRepel, $(obs_data)[^5_1], $(obs_data)[^5_2])
plot!(fig[1,1], spec)
```

When `obs_data` updates, Makie diffs the spec and **reuses the plot object if the type matches** rather than destroying and rebuilding it. The reactive solver chain inside the recipe then fires naturally.[^5_3]

The catch: SpecApi's diff logic updates *attributes*, but your bbox spy requires a **first-render pass** to populate. This is fine for static figures, but if `obs_data` changes significantly (new number of labels), the ghost `textlabel!` needs to re-fire. You'd handle this by including the label count in the Observable dependency chain so the bbox store is invalidated and rebuilt on structural changes.

***

## Key Open Questions Before You Write Code

1. **When exactly does the spy shape callback fire in each backend?** CairoMakie is synchronous (immediate on `display`); GLMakie is asynchronous (deferred to GPU render loop). You may need `on(events(scene).tick)` to know when the first frame has rendered in GLMakie before reading captured bboxes.
2. **Is `Makie.project` stable API?** It's used internally but not documented as public. Worth checking `Makie.to_ndim` and `Makie.project` signatures across v0.21–0.24.
3. **Connector line clipping.** ggrepel clips the segment to the label box edge (not center). Worth deciding early if you want geometric edge intersection or just center-to-anchor.
<span style="display:none">[^5_10][^5_11][^5_12][^5_13][^5_14][^5_15][^5_4][^5_5][^5_6][^5_7][^5_8][^5_9]</span>

<div align="center">⁂</div>

[^5_1]: https://discourse.julialang.org/t/makie-does-not-update-the-plot-during-an-animation/91109

[^5_2]: https://docs.makie.org/v0.22/explanations/recipes

[^5_3]: https://docs.makie.org/dev/explanations/specapi

[^5_4]: https://docs.makie.org/dev/explanations/observables

[^5_5]: https://discourse.julialang.org/t/having-a-hard-time-with-makie-recipes/119978

[^5_6]: https://discourse.julialang.org/t/automatic-update-in-makie-with-observable-of-vector/85623

[^5_7]: https://docs.makie.org/dev/tutorials/wrap-existing-recipe

[^5_8]: https://book.jinguo-group.science/stable/append/plotting/

[^5_9]: https://docs.makie.org/v0.21/explanations/recipes

[^5_10]: https://discourse.julialang.org/t/makie-update-plot-legend-through-observables/86786

[^5_11]: https://juliadynamics.github.io/Agents.jl/v4.3/interact/

[^5_12]: https://github.com/MakieOrg/Makie.jl/issues/837

[^5_13]: https://www.reddit.com/r/Julia/comments/1hnckpq/updating_a_scatteplot_in_glmakie/

[^5_14]: https://github.com/MakieOrg/Makie.jl/issues/552

[^5_15]: https://discourse.julialang.org/t/makie-jl-keyword-argument-recipes-v0-24/132603


---

# Great thanks!

You're welcome! You've got a solid foundation to work from — the spy-shape bbox capture is a clever way to stay on public API, and modeling the solver as a pure Observable transformation means zoom/resize reactivity comes for free.

When you're ready to start coding, the natural first milestone would be getting the bbox spy working in isolation on a simple `textlabel!` call before wiring up the full recipe. Good luck with it — would be a genuinely useful package for the Julia plotting ecosystem.
