# Makie `annotation!` recipe — core surface area

**Source:** `/home/jonathanchen/.julia/packages/Makie/p9K7f/src/basic_recipes/annotation.jl` (Makie 0.24.10, 1190 lines)
**Rendered docs cross-checked:** `https://docs.makie.org/stable/reference/plots/annotation`
**ComputePipeline version:** 0.1.7 (`/home/jonathanchen/.julia/packages/ComputePipeline/30b0T/`)

---

## Abstract

The `Annotation` recipe is a *two-positional-argument* recipe taking
`(label_offsets_or_positions::Vector{<:Vec2}, target_positions::Vector{<:Point2})`.
All five public call forms collapse to this pair through `convert_arguments`
(annotation.jl:197-229). The recipe internally builds a `Text` plot for the
labels and a `plotlist!` for the connection geometry. The label placement is
handled by a *single override-able function* — `calculate_best_offsets!` —
which receives all data in **pixel space** (regardless of `labelspace`),
mutates an offset vector in-place, and supports a NaN sentinel for "this
label is fully auto" vs "this label has a manual offset/position". The
compute graph has 18 attributes plus six internal computed nodes
(`computed_textcolor`, `screenpoints_target`, `text_bbs`, `screenpoints_label`,
`offsets`, `plotspecs`). A hidden `__advance_optimization` input (annotation.jl:290)
lets the user resume optimization without resetting, which `advance_optimization!`
(annotation.jl:361) drives. Two algorithm types ship: `Automatic` (a thin
dispatch that checks the NaN sentinel then delegates) and `LabelRepel`
(the actual force-directed solver in annotation.jl:454-531) with hard-coded
`repel=0.1`, `attract=0.1`, `padding=(6,5) px` defaults. The viewport
acts as a hard clamp inside the algorithm (annotation.jl:512-528).

---

## 1. Signatures and call forms

### 1.1 The recipe declaration

annotation.jl:95 declares the recipe with a typed argument tuple:

```julia
@recipe Annotation (label_offsets_or_positions::Vector{<:Vec2}, target_positions::Vector{<:Point2}) begin
    ...
end
```

This is processed by `create_recipe_expr` (recipes.jl:474) and
`create_args_type_expr` (recipes.jl:382-406). Because the field types are
provided, Makie emits a `types_for_plot_arguments(::Type{<:Annotation})`
returning `Tuple{Vector{<:Vec2}, Vector{<:Point2}}` and an `argument_names`
implementation `(:label_offsets_or_positions, :target_positions)`. After
`convert_arguments`, these become the two plot inputs `p.label_offsets_or_positions`
and `p.target_positions` in the compute graph.

The recipe macro also generates `annotation(args...; kw...)` and
`annotation!(args...; kw...)` (recipes.jl:528-535) which both call
`_create_plot[!]`. There is no `Annotation3` variant — annotation is 2D-only.

### 1.2 Public call forms (`convert_arguments` overloads)

There are seven `convert_arguments` methods (annotation.jl:197-229). All produce
the canonical pair `(Vector{Vec2d}, Vector{Point2d})`:

| Form | Definition | Behavior |
|------|------------|----------|
| `annotation(x::Real, y::Real)` | annotation.jl:197 | `[Vec2d(NaN)], [Vec2d(x, y)]` — single target at `(x,y)`, label fully auto. |
| `annotation(p::VecTypes{2})` | annotation.jl:201 | `[Vec2d(NaN)], [Point2d(p...)]` — single target as a 2-vector. |
| `annotation(x, y, x2, y2)` (all `Real`) | annotation.jl:205 | `[Vec2d(x, y)], [Point2d(x2, y2)]` — label at `(x,y)`, target at `(x2,y2)`, **manual placement** (no NaN). |
| `annotation(p1::VecTypes{2}, p2::VecTypes{2})` | annotation.jl:209 | `[Vec2d(p1...)], [Point2d(p2...)]` — manual placement with point pairs. |
| `annotation(v::AbstractVector{<:VecTypes{2}})` | annotation.jl:213 | `fill(Vec2d(NaN), N), Point2d.(...)` — vector of targets, all auto. |
| `annotation(v1::AbstractVector{<:VecTypes{2}}, v2::AbstractVector{<:VecTypes{2}})` | annotation.jl:218 | Two parallel vectors → manual placement of `N` labels at `v1[i]` to targets `v2[i]`. |
| `annotation(xs::Vector{<:Real}, ys::Vector{<:Real})` | annotation.jl:222 | `fill(Vec2d(NaN), N), Point2d.(xs, ys)` — vector form using parallel scalar arrays, all auto. |
| `annotation(x1s, y1s, x2s, y2s)` (all `Real` vectors) | annotation.jl:227 | Four parallel arrays → manual placement. |

**Auto vs manual mode** is therefore decided per-call by the input form. The
"auto" forms write `Vec2d(NaN)` into every slot of `label_offsets_or_positions`,
which is later interpreted as a sentinel inside `calculate_best_offsets!`
(annotation.jl:443: `if all(!isnan, textpositions_offset)` — manual short-circuit).

**Important corollary:** there is **no per-label mixed mode** at the public API
level. You either get all-NaN or all-finite from `convert_arguments`. The
`calculate_best_offsets!` core check is `all(!isnan, ...)` — if *any* element
is NaN, the algorithm engages on *all* labels and the manual labels lose their
manual placement. The TODO at annotation.jl:447-449 acknowledges this:

```julia
# TODO: make it so some positions can be fixed and others are not (NaNs)
# giving one component of the position could be cool, like only x in data space, but this
# doesn't really work because projection into screen space needs x and y together
```

A consumer wanting mixed mode has to construct the input vector manually
(e.g. `annotation!(ax, mixed_offsets, targets)`) and provide a custom algorithm
that honors per-slot NaNs.

### 1.3 The two-argument internal shape

After conversion, the recipe always sees:

* `p.label_offsets_or_positions :: Vector{Vec2d}` — the user-supplied label
  anchors. In **manual** mode, these are interpreted as positions (either in
  pixels relative to target, or in data, depending on `labelspace`). In
  **auto** mode (`NaN`s), they are the input to the placement algorithm but
  are bypassed (only the NaN check matters).
* `p.target_positions :: Vector{Point2d}` — the data-space points being annotated.

Note the asymmetry: targets are always **points in data**; offsets-or-positions
are *either* offsets (in `:relative_pixel`) *or* positions (in `:data`),
controlled by the `labelspace` attribute (see §6).

---

## 2. Complete attribute surface

The `@recipe Annotation` block (annotation.jl:96-173) declares **18 attributes**.
Every one has a docstring inside the recipe block (the docs page is auto-generated
by `make_recipe_docstring` at recipes.jl:560-583). Below, each attribute is
documented with name, default, expected type, semantic role, and any
non-obvious behavior.

### 2.1 Text/font attributes (forwarded to inner `text!`)

| Attribute | Default | Type | Role |
|-----------|---------|------|------|
| `textcolor` | `automatic` | Color or `Automatic` | annotation.jl:99. Color of the text labels. When `automatic`, falls back to `color` via the `computed_textcolor` node (annotation.jl:236, using `default_automatic` from utilities.jl:41). |
| `text` | `""` | `String` / `Vector{String}` / rich-text broadcasted | annotation.jl:107. Forwarded as-is to `text!` (annotation.jl:245). |
| `font` | `@inherit font` | `Symbol` or `String` | annotation.jl:111. Looked up via the theme's `fonts` dict if `Symbol`. |
| `fonts` | `@inherit fonts` | dict-like | annotation.jl:115. Theme font dictionary, e.g. `:regular`, `:bold`, `:italic`. |
| `fontsize` | `@inherit fontsize` | `Float64` | annotation.jl:119. |
| `align` | `(:center, :center)` | `Tuple{Symbol, Symbol}` or numeric | annotation.jl:123. Text alignment relative to the label anchor. |
| `justification` | `automatic` | `:left` / `:center` / `:right` / fraction / `Automatic` | annotation.jl:127. Defaults to horizontal `align`. |
| `lineheight` | `1.0` | `Float64` | annotation.jl:131. |

### 2.2 Placement / algorithm attributes

| Attribute | Default | Type | Role |
|-----------|---------|------|------|
| `labelspace` | `:relative_pixel` | `Symbol` (`:relative_pixel` or `:data`) | annotation.jl:163. Interpretation of `label_offsets_or_positions`. See §6. |
| `maxiter` | `automatic` | `Union{Automatic, Int}` | annotation.jl:156. Cap on solver iterations. `Automatic` resolves to **200** inside `LabelRepel` (annotation.jl:460). |
| `algorithm` | `automatic` | algorithm tag (`Automatic` or any value with a `calculate_best_offsets!` method) | annotation.jl:170. The dispatch hook — the *only* place a third-party label-placer plugs in. |

### 2.3 Path / connection style

| Attribute | Default | Type | Role |
|-----------|---------|------|------|
| `path` | `Ann.Paths.Line()` | `Ann.Paths.Line` / `Ann.Paths.Corner` / `Ann.Paths.Arc` / vector | annotation.jl:136. The shape of the connector. See §2.6. |
| `style` | `automatic` | `Ann.Styles.Line` / `Ann.Styles.LineArrow` / `Ann.Styles.WithText` / vector / `Automatic` | annotation.jl:141. How the path is rendered. `Automatic` → `Ann.Styles.Line()` (annotation.jl:1013). |
| `shrink` | `(5.0, 7.0)` | `Tuple{Real,Real}` or vector of tuples | annotation.jl:147. Radii in **pixels** of clipping circles at start (label end) and stop (target end) of the path. Adds visual breathing room. Applied via `shrink_path` (annotation.jl:668-724). |
| `clipstart` | `automatic` | a `Rect2`, or `Automatic` | annotation.jl:152. The shape used to clip the start of the path. `automatic` uses the text bounding box. Applied via `clip_path_from_start` (annotation.jl:845). |
| `linewidth` | `1.0` | `Float64` | annotation.jl:165. Default line width for the path. Style-level `linewidth` (e.g. on `Ann.Arrows.Line.linewidth`) overrides via `_auto` (annotation.jl:1067). |

### 2.4 Color and misc

| Attribute | Default | Type | Role |
|-----------|---------|------|------|
| `color` | `@inherit linecolor` | Color | annotation.jl:103. Base color of the connector. Style objects can override per-piece. Also feeds `textcolor` when that is `automatic`. |
| `visible` | `true` | `Bool` | annotation.jl:172. Hides the plot. There is a TODO at annotation.jl:355 noting that **dynamic** updates to `visible` don't currently propagate through `plotlist!`, only the initial value is honored (the recipe passes `p.visible[]`, deref'd at construction time). |

### 2.5 Hidden / internal-only attributes

The recipe adds two more attributes to `p.attributes` *after* the recipe macro
expansion, neither of which appears in the docs page or the `@recipe` block:

| Attribute | Default | Type | Role |
|-----------|---------|------|------|
| `viewport` | bound to `parent_scene(p).compute[:viewport]` | `Rect2i` | annotation.jl:289. The scene's viewport rectangle — used as the `bbox` clamp passed to the algorithm. |
| `__advance_optimization` | `0` | `Int` | annotation.jl:290. A "kick the solver forward N steps without resetting" signal. The user calls `advance_optimization!(p, n)` (annotation.jl:361-365). Distinguished inside the offsets computation by being the *only* changed input (annotation.jl:300). The leading double underscore signals internal use. |
| `space` | `:data` | `Symbol` | Added via `add_constant!(p.attributes, :space, :data)` at annotation.jl:257. The recipe is always rooted in data space (its targets are data points). This is consumed by `register_projected_positions!` indirectly through plot machinery. |

### 2.6 The `Ann` baremodule — path & style objects

annotation.jl:1-78 declares `baremodule Ann` (with sub-modules `Paths`, `Arrows`,
`Styles`) — a barebones namespace chosen for "cleanest tab-completion
behavior" per the comment on annotation.jl:1.

**`Ann.Paths`** (annotation.jl:4-14):

| Type | Fields | Notes |
|------|--------|-------|
| `Ann.Paths.Line` | (none) | Straight line. `connection_path` → single `LineTo` (annotation.jl:588). |
| `Ann.Paths.Corner` | (none) | One right-angle bend; orientation chosen by dominant axis of the displacement (annotation.jl:569-583, 597-616). |
| `Ann.Paths.Arc` | `height::Float64 = 0.5` | Bezier arc. Positive = arcs up-then-down, negative = down-then-up; magnitude `1` ≈ half circle. annotation.jl:10-12, with center computed in `arc_center_radius` (annotation.jl:641-660). `connection_path` produces a `BezierPath` with an `EllipticalArc` (annotation.jl:662-666). If `abs(height) < 1e-4` it degrades to `Line` (annotation.jl:663). |

**`Ann.Arrows`** (annotation.jl:16-35):

| Type | Fields | Notes |
|------|--------|-------|
| `Ann.Arrows.Line` | `length::Float64=8.0`, `angle::Float64=deg2rad(60)`, `color=automatic`, `linewidth::Union{Automatic,Float64}=automatic` | Open V-shape arrowhead (no fill). `plotspecs` at annotation.jl:1076-1087. |
| `Ann.Arrows.Head` | `length::Float64=8.0`, `angle::Float64=deg2rad(60)`, `color=automatic`, `notch::Float64=0` | Filled triangular arrowhead with optional concave back. `plotspecs` at annotation.jl:1089-1101 — implemented as a `Scatter` with a custom `BezierPath` marker. `shrinksize(l::Head) = l.length * (1 - l.notch)` (annotation.jl:1072-1074) — the head also shortens the path so the line doesn't poke through it. |

**`Ann.Styles`** (annotation.jl:37-77):

| Type | Fields | Notes |
|------|--------|-------|
| `Ann.Styles.Line` | (none) | Plain line, no arrowheads. |
| `Ann.Styles.LineArrow` | `head=Arrows.Line()`, `tail=nothing` | Line plus optional head and tail. Either can be `nothing` to skip. `annotation_style_plotspecs` at annotation.jl:1015-1045. |
| `Ann.Styles.WithText` | `style`, `text`, `fontsize=12.0`, `align=(:center,:bottom)`, `offset=4.0`, `color=automatic` | Layers a `PathText` along the connector on top of an inner style. annotation.jl:51-76, 1053-1065. Color falls back to the outer `color` when `automatic`. |

Each of these may be **scalar or vector** at the recipe level — the `broadcast_foreach`
at annotation.jl:334 handles per-label dispatch.

---

## 3. Compute-graph wiring

`plot!(p::Annotation)` lives at annotation.jl:235-359. It registers the
following nodes, in order:

### 3.1 `computed_textcolor` (annotation.jl:236)

```julia
map!(default_automatic, p, [:textcolor, :color], :computed_textcolor)
```

A two-input computed node. `default_automatic` (utilities.jl:41) returns
the first argument unless it is `automatic`, in which case it returns the
second. So `computed_textcolor = textcolor` unless `textcolor === automatic`,
in which case `computed_textcolor = color`. Fed into the inner `text!`.

### 3.2 Inner `text!` plot (annotation.jl:238-251)

A child `Text` plot is created at the **target positions** with an `offset`
input pre-filled with **zeros** (`zeros(Vec2f, length(p.target_positions[]))`).
The actual offset gets pushed in later via an `on(offsets -> update!(...))`
observable. The text plot inherits font, fonts, fontsize, justification,
lineheight, visible from `p`; color goes through `computed_textcolor`.

A key consequence: the **text is drawn at the target position, then offset by
pixels to the resolved label location**. The label's anchor in data space is
always the target; the label moves around it.

### 3.3 `txt.raw_string_boundingboxes` (text.jl:586-616)

The recipe forces the inner Text plot to expose per-string bounding boxes by
calling `register_raw_string_boundingboxes!(txt)` (annotation.jl:255).
These boxes are in **markerspace** (which for `text!` is pixel-space) and
**exclude** the `offset` and per-string `position`, *but include* font rotation
and inter-line layout. The recipe deliberately doesn't include `offsets` here
because input length changes would error during transient resize windows
(annotation.jl:253-254 comment).

### 3.4 `screenpoints_target` (annotation.jl:258-261)

```julia
register_projected_positions!(
    p, Point2f, input_name = :target_positions,
    output_name = :screenpoints_target, output_space = :pixel
)
```

Targets projected through (transform_func → model → f32convert → projection
matrix → yflip handling) to pixel space. The mechanics are in
projection_utils.jl:55-160. This is the canonical "where on screen is each
target".

### 3.5 `text_bbs` (annotation.jl:263-265)

```julia
map!(p, [txt.raw_string_boundingboxes, p.screenpoints_target], :text_bbs) do bboxes, px_pos
    return _guard_nonfinite.(Rect2d.(bboxes)) .+ px_pos
end
```

Each label's pixel-space bounding box, anchored at its **target's pixel
position**. `_guard_nonfinite` (annotation.jl:233) replaces non-finite rects
(empty strings produce `Rect3d()` with `Inf` widths) with a zero-size rect at
the origin so downstream math doesn't blow up. The `+ px_pos` shifts the
bbox so it sits at the target — i.e. these are the bboxes the labels *would*
have if no offset were applied.

### 3.6 `register_camera_matrix!(p, :data, :pixel)` (annotation.jl:267)

This installs the `:world_to_pixel` matrix in `p.attributes`
(`:data` aliases to `:world` per camera.jl:240-243; `:world_to_pixel` is set
up at camera.jl:199-202). Used in the labelspace=`:data` branch below.

### 3.7 `screenpoints_label` (annotation.jl:268-287)

The most logic-dense node:

```julia
inputs = [
    :screenpoints_target, :labelspace, :label_offsets_or_positions,
    :world_to_pixel, :f32c, :model, :transform_func,
]
register_computation!(
    p.attributes, inputs, [:screenpoints_label]
) do (tps, space, loffpos, proj, f32c, model, tf), changed, cached
    if space === :relative_pixel
        if isnothing(cached) || changed[1] || changed[2] || changed[3]
            return (tps .+ loffpos,)
        else
            # Skip updates from camera and transform func
            return (nothing,)
        end
    else
        transformed_label_pos = apply_transform(tf, loffpos)
        f32c_mat = f32_convert_matrix(f32c)
        return (_project(Point2f, proj * f32c_mat * model, transformed_label_pos),)
    end
end
```

In `:relative_pixel` mode, `screenpoints_label = screenpoints_target + label_offsets`
in pixel coordinates. The clever bit is that this node **deliberately ignores
camera-induced changes** (return `(nothing,)`) — if only `world_to_pixel`,
`f32c`, `model`, or `transform_func` changed, the cached value is kept. This
prevents the optimizer from churning on pan/zoom in `relative_pixel` mode.

In `:data` mode, the offsets are treated as data-space *positions* and
projected through the full transform stack to pixel space. Camera changes
*do* propagate here because the matrices are inputs to the projection.

### 3.8 `viewport` and `__advance_optimization` inputs (annotation.jl:289-290)

```julia
add_input!(p.attributes, :viewport, parent_scene(p).compute[:viewport])
add_input!(p.attributes, :__advance_optimization, 0)
```

The viewport is hooked to the scene's viewport observable so the algorithm's
clamping rectangle updates on resize. `__advance_optimization` starts at 0.

### 3.9 `offsets` (annotation.jl:294-322)

The single biggest computation in the pipeline:

```julia
inputs = [
    :algorithm, :screenpoints_target, :screenpoints_label, :text_bbs,
    :viewport, :labelspace, :maxiter, :__advance_optimization,
]
register_computation!(p.attributes, inputs, [:offsets]) do args, changed, cached
    # We should only advance if it's the only thing causing an update?
    advance = sum(values(changed)) == 1 && changed.__advance_optimization

    # Probably required when input sizes change?
    offsets = isnothing(cached) ? Vec2f[] : cached[1]
    resize!(offsets, length(args.screenpoints_target))

    calculate_best_offsets!(
        args.algorithm,
        offsets,
        args.screenpoints_target,
        args.screenpoints_label,
        args.text_bbs,
        Rect2d((0, 0), widths(args.viewport));
        labelspace = args.labelspace,
        maxiter = ifelse(advance, args.__advance_optimization, args.maxiter),
        reset = !advance,
    )

    return (offsets,)
end
```

Key observations:

* **`advance` semantics** (annotation.jl:300): the recipe only counts it as an
  "advance" step if `__advance_optimization` is the *sole* changed input. If
  literally anything else changed in the same pulse, the solver is reset.
  This means `advance_optimization!` cannot meaningfully chain through other
  attribute changes — but it is the right behavior to avoid silent
  optimization persistence across data changes.
* **`reset = !advance`** (annotation.jl:318): in the normal case, the
  algorithm is asked to zero the offsets array before running. In advance
  mode, it preserves them.
* **`maxiter` override** (annotation.jl:314): in advance mode, the iteration
  count comes from `__advance_optimization` itself (so `advance_optimization!(p, 5)`
  runs 5 more iterations). In normal mode, `args.maxiter` is used.
* **Resize on input-size change** (annotation.jl:304): the offsets vector is
  resized to match `length(screenpoints_target)`. This means adding/removing
  labels works, but the first iteration with new entries starts at zero
  (since `reset` will likely be true).
* **`bbox` for the algorithm** is `Rect2d((0,0), widths(viewport))` — the
  viewport translated to origin-at-0. This is in pixel space.

### 3.10 Offsets → text plot observable (annotation.jl:326)

```julia
on(offsets -> update!(txt, offset = offsets), p.offsets, update = true)
```

A regular Observables.jl `on` callback, *not* a compute-graph edge. The
comment at annotation.jl:324-325 explains: "create observable updating
offsets in text plot. This forces everything offsets rely on to update asap,
before the backend pulls." It is fired immediately on initial registration
(`update = true`).

### 3.11 `plotspecs` (annotation.jl:328-353)

```julia
inputs = [
    :text_bbs, :screenpoints_target, :offsets, :path, :clipstart, :shrink,
    :style, :color, :linewidth,
]
map!(p, inputs, :plotspecs) do text_bbs, points, offsets, path, clipstart, shrink, style, color, linewidth
    specs = PlotSpec[]
    broadcast_foreach(text_bbs, points, clipstart, offsets) do text_bb, p2, clipstart, offset
        offset_bb = text_bb + offset

        p2 in offset_bb && return  # target inside label — skip connector
        p1 = startpoint(path, offset_bb, p2)
        _path = connection_path(path, p1, p2)

        clipstart = if clipstart === automatic
            offset_bb
        else
            clipstart
        end
        clipped_path = clip_path_from_start(_path, clipstart)
        shrunk_path = shrink_path(clipped_path, shrink)

        append!(specs, annotation_style_plotspecs(style, shrunk_path, p1, p2; color, linewidth))
    end
    return specs
end
```

This is the **connector rendering pipeline** — it doesn't touch label
placement; it consumes `offsets` and emits per-annotation `PlotSpec`s.
Per-annotation flow:

1. Compute the offset bounding box (`text_bb + offset`).
2. **Skip rendering** if the target falls inside the label box (annotation.jl:337:
   `p2 in offset_bb && return`).
3. Pick a start point on the label box (path-shape-dependent — `startpoint(::Line, ...)`
   returns the *center* of the box (annotation.jl:567); `Corner` picks an edge
   midpoint based on direction (annotation.jl:569-583); `Arc` returns the
   center (annotation.jl:618-620)).
4. Build the connection path geometrically.
5. Clip from start using `clipstart` (the box, unless overridden).
6. Apply `shrink` (circle-radius clips at both ends).
7. Hand off to the style for final `PlotSpec` generation.

The output `plotspecs` is fed to `plotlist!(p, p.plotspecs; visible = p.visible[])`
(annotation.jl:356). The `visible[]` deref is the TODO at annotation.jl:355
("passing dynamic attributes doesn't work (visible)").

### 3.12 Compute-graph dependency summary

```
text → text_blocks, raw_glyph_bbs, ...  (inside child Text plot)
                              ↓
          raw_string_boundingboxes (text plot, pixel space)
                              ↓ (+screenpoints_target)
                          text_bbs
                              ↓
                                            ┌──→ screenpoints_label ──┐
target_positions ──→ screenpoints_target ───┤                          │
                                            │  label_offsets_or_pos ──┤
                                            │  labelspace ────────────┤
                                            │  world_to_pixel, f32c,  │
                                            │  model, transform_func  │
                                            └─────────────────────────┘
                                                      ↓
            algorithm, viewport, maxiter, __advance_optimization → offsets
                                                      ↓
                            (Observables.on) ────────→ updates txt.offset
                                                      ↓
text_bbs, screenpoints_target, offsets, path, clipstart, shrink, style, color, linewidth
                                                      ↓
                                                  plotspecs
                                                      ↓
                                                plotlist! (connectors)
```

### 3.13 `advance_optimization!` API

annotation.jl:361-365:

```julia
function advance_optimization!(p::Annotation, n::Int = 1)
    @assert n > 0
    p.__advance_optimization = n
    return
end
```

Pure attribute write; the assignment fires the `offsets` computation with
`__advance_optimization` as the sole changed input, which the node detects
and treats as "continue from current state for N more iterations".

---

## 4. The `calculate_best_offsets!` algorithm contract

### 4.1 Top-level dispatch (`Automatic`) — annotation.jl:421-452

```julia
function calculate_best_offsets!(
        ::Automatic, offsets::Vector{<:Vec2}, textpositions::Vector{<:Point2}, textpositions_offset::Vector{<:Point2}, text_bbs::Vector{<:Rect2}, bbox::Rect2;
        maxiter::Union{Automatic, Int},
        reset::Bool,
        labelspace::Symbol,
    )
    if !(length(offsets) == length(textpositions) == length(textpositions_offset) == length(text_bbs))
        error("Mismatching array sizes: ...")
    end

    if reset
        offsets .= zero.(eltype(offsets))
    end

    if all(!isnan, textpositions_offset)
        offsets .= textpositions_offset .- textpositions
        return
    end
    # TODO: make it so some positions can be fixed and others are not (NaNs)

    return calculate_best_offsets!(LabelRepel(), offsets, textpositions, textpositions_offset, text_bbs, bbox; maxiter, labelspace)
end
```

This is the manual-vs-auto switch:

* Resets `offsets` to zero if asked.
* If **none** of `textpositions_offset` contain a NaN, computes
  `offsets = textpositions_offset - textpositions` and returns — full manual
  mode, no iteration.
* Otherwise delegates to `LabelRepel` (the default solver).

### 4.2 Arguments (positional)

The full positional signature of any `calculate_best_offsets!` method:

| Position | Name | Type | Frame | Mutability |
|----------|------|------|-------|------------|
| 1 | `algorithm` | dispatch tag (any) | — | read-only |
| 2 | `offsets` | `Vector{<:Vec2}` (`Vec2f` in practice) | **pixel** (output frame) | **mutated in place** — the algorithm's job is to fill this |
| 3 | `textpositions` | `Vector{<:Point2}` (`Point2f`) | **pixel** | read-only (these are `screenpoints_target`) |
| 4 | `textpositions_offset` | `Vector{<:Point2}` (`Point2f`) | **pixel** | read-only (these are `screenpoints_label` — the *initial* / *intended* label positions, in pixels) |
| 5 | `text_bbs` | `Vector{<:Rect2}` (`Rect2d` here) | **pixel**, anchored at target | read-only |
| 6 | `bbox` | `Rect2` (`Rect2d`) | **pixel**, viewport at origin | read-only — soft viewport clamp |

### 4.3 Keyword arguments

| Name | Type | Meaning |
|------|------|---------|
| `maxiter` | `Union{Automatic, Int}` | Algorithm-defined; `LabelRepel` resolves `automatic` to 200 (annotation.jl:460). |
| `reset` | `Bool` | When true, the *outer* dispatch zeros the offsets before delegating. The `LabelRepel` inner method does not receive `reset`. |
| `labelspace` | `Symbol` | `:relative_pixel` or `:data`. Currently `LabelRepel` doesn't use it — but it is *passed* to the algorithm, which is the extension point for algorithms that want different policies per space. |

### 4.4 What an algorithm is allowed to mutate

* **`offsets`** — must be written in-place. The returned value is ignored;
  the recipe reads `args.offsets` after the call (this is the same array that
  was cached by the compute node — annotation.jl:303).
* **Nothing else.** The other positional arguments (`textpositions`,
  `textpositions_offset`, `text_bbs`, `bbox`) are *not* documented as mutable
  and the `LabelRepel` implementation only reads them.

### 4.5 What the algorithm doesn't know

* It does *not* know which labels were originally manual vs auto — the
  outer dispatch has already filtered those cases. By the time the algorithm
  is called, the input is "treat all labels as movable".
* It does not have access to the inner `text!` plot, the camera, the
  scene, or the user-facing plot object. The interface is pure data.
* It does not get a global iteration counter or know how many times it has
  been called.

### 4.6 The `LabelRepel` solver — annotation.jl:415-531

A 100-line force-directed iterative solver:

```julia
Base.@kwdef struct LabelRepel
    repel::Float64 = 0.1     # strength of all repulsive terms
    attract::Float64 = 0.1   # strength of attraction to own target
    padding::Vec2d = Vec2d(6, 5)  # pixel padding around each text bbox (per side)
end
```

The constants `repel = attract = 0.1` and `padding = (6, 5) px` are
**hard-coded defaults** — there is no exposed knob to tune them from the
user side without constructing a `LabelRepel` and passing it via
`algorithm = LabelRepel(repel=..., ...)`. This is the *only* way to tune.

Per iteration (annotation.jl:479-529):

1. **Recompute padded offset bboxes:** `offset_bbs .= padded_bbs .+ offsets`.
2. **Pairwise label repulsion:** for each pair `(i, j)`, compute
   `overlap = repel * rect_overlap(bb_i, bb_j)` (annotation.jl:550-565 — the
   minimum-cost separation vector), push `i` back by `-overlap`, push `j` forward by
   `+overlap`.
3. **Attraction to own target:** if the target is *outside* the padded box,
   pull the box toward it with `algorithm.attract * distance_point_outside_rect(target, bb)`
   (annotation.jl:367-388).
4. **Repulsion from *all* other targets:** for each label `i`, for each
   target `j` (including `i`'s own), if target `j` is *inside* the box,
   push the box away by `algorithm.repel * distance_point_inside_rect(target, bb)`
   (annotation.jl:390-413, which picks the smaller of x/y distance — "faster"
   escape direction).
5. **Hard viewport clamp** (annotation.jl:512-528): after force application,
   clamp `padded_bbs[i] + offsets[i]` to lie inside `bbox`. The clamp is
   *axis-independent* (handles x and y separately), so a box pushed past two
   walls only gets pinned on one axis worth of correction per step — the
   solver may take several iterations to settle into a corner.

Pre-loop one-shot **centering bias** (annotation.jl:469-477): if all offsets
are zero on entry, each label is pre-nudged toward the viewport center by
`0.1 * algorithm.repel / norm(v) * v` so the self-repulsion forces don't
trap things at edges.

**Notable issues / TODOs visible in this algorithm:**

* The own-point repulsion at annotation.jl:502-509 iterates `j in 1:length(textpositions)` — i.e.
  *includes* the label's own target. This is the canonical "self-repel" bug
  that the design spec at `docs/superpowers/plans/2026-05-27-own-point-repulsion.md`
  in this repo highlights as needing redesign — Makie always repels a label
  from its own target. The attraction step above counterbalances this only
  when the target is outside the box.
* The viewport clamp does not respect axis limits (the README says axis-limit
  clamping is a TextRepel-specific fix; Makie's annotation clamps to the
  *viewport*, which is the whole drawing area including subplot gutters).
* There is no convergence detection — every call runs the full `maxiter`
  iterations.
* The function returns `nothing`; the recipe never inspects the return value.

### 4.7 The custom-algorithm contract — practical recipe

For an external algorithm (e.g. ours) to plug in:

```julia
struct MyAlgo  # any type
    ...
end

function Makie.calculate_best_offsets!(
        algo::MyAlgo,
        offsets::Vector{<:Vec2},
        textpositions::Vector{<:Point2},
        textpositions_offset::Vector{<:Point2},
        text_bbs::Vector{<:Rect2},
        bbox::Rect2;
        maxiter::Union{Makie.Automatic, Int},
        reset::Bool,           # <-- if you want to play nicely with advance_optimization!
        labelspace::Symbol,    # <-- can ignore if you don't care
    )
    # 1. honor `reset` if you want
    # 2. mutate `offsets` in place; final layout is `textpositions[i] + offsets[i]` in pixels
    # 3. return value is discarded
    return
end
```

User invokes via `annotation!(...; algorithm = MyAlgo(...))`.

**Important contract subtleties:**

1. **`maxiter::Union{Automatic, Int}` not `::Int`** — you must accept
   `Makie.automatic` (the singleton) and pick your own default.
2. **`reset::Bool` is a required kw** — the recipe always passes it. If your
   algorithm signature omits it, dispatch fails on calls without `reset`.
   It's safe to take it and ignore it.
3. **`labelspace::Symbol` is required** — same as above.
4. **All inputs are in pixel space** regardless of `labelspace`. The recipe
   has already projected. Your algorithm produces *pixel* offsets which the
   inner `text!` consumes as a pixel `offset` attribute.
5. **`text_bbs` are anchored at the *target* pixel position**, not at zero
   and not at the originally-requested label position. The current label
   position before optimization is `textpositions_offset[i]`, which equals
   `text_bbs[i].origin + 0.5*widths(text_bbs[i])` only when `align=(:center,:center)`
   and there is no prior offset. In general, `textpositions_offset[i]` is the
   anchor point and `text_bbs[i]` is the bbox the recipe wants the algorithm
   to think of as "where the text would be drawn if `offset == 0`".

   That last point is subtle — see §4.8.

### 4.8 The "what does `text_bbs` mean exactly" gotcha

annotation.jl:263-265 builds `text_bbs` as:

```julia
text_bbs = _guard_nonfinite.(Rect2d.(raw_string_boundingboxes)) .+ screenpoints_target
```

`raw_string_boundingboxes` (text.jl:586-616) are the per-string bboxes in
markerspace **without** the position offset but **with** alignment built in
(`align` shifts the glyph origins inside the bbox). The Text recipe applies
`offset` *after* placement.

So `text_bbs[i]` is "the rect the label would occupy if it were drawn at its
target with zero pixel-offset, given its current `align`". This is a *valid*
starting bbox, but it's almost certainly **overlapping the target** when
`align = (:center, :center)`.

`textpositions_offset[i]` is "the rect's anchor position if labelspace is
relative_pixel and no NaNs" — i.e. the position where the label *should*
end up before any optimization is applied. For NaN inputs (auto mode),
`textpositions_offset == textpositions` (zero offset) because
`tps .+ loffpos == tps + NaN`. **Wait** — there's a subtlety here. In
relative_pixel mode (annotation.jl:276-282):

```julia
return (tps .+ loffpos,)
```

If `loffpos` has NaNs (auto mode), then `screenpoints_label == NaN` too. That
makes `textpositions_offset` (NaN points) in the algorithm input. The
`all(!isnan, textpositions_offset)` check at annotation.jl:443 detects the
manual case; in the auto case, `textpositions_offset` is mostly NaN — which
means *the algorithm cannot use it as an initial position*. It should only
use it for the manual short-circuit check or for per-slot NaN detection
(which Makie doesn't yet support).

In the **data labelspace** auto case, the conversion uses `Vec2d(NaN)` too —
they go through `apply_transform(tf, NaN)` and `_project(..., NaN)`, which
should produce NaN outputs too.

**Net effect for a custom algorithm:** `textpositions_offset` is essentially
useless for initialization in auto mode — you must rely on `textpositions`
plus your own seeding strategy. `LabelRepel` does this implicitly by
starting from zero offsets and biasing toward the viewport center
(annotation.jl:469-477).

---

## 5. NaN-sentinel semantics, `reset`, and manual/auto modes

### 5.1 The NaN sentinel in `label_offsets_or_positions`

`Vec2d(NaN)` (a 2-vector of NaNs) is the sentinel for "no manual position /
fully automatic for this slot". It is written by `convert_arguments` in all
the "one-vector" forms (annotation.jl:198, 202, 215, 224).

The sentinel is consumed at *one* place: annotation.jl:443.

```julia
if all(!isnan, textpositions_offset)
    offsets .= textpositions_offset .- textpositions
    return
end
```

* **`all(!isnan, ...)` is true ⇒** every label has a manual position (the
  outer two-vector forms). The algorithm short-circuits and uses
  `offset = position − target` (in pixels).
* **`all(!isnan, ...)` is false ⇒** at least one NaN — *the entire batch* is
  treated as fully auto. The algorithm runs on every label.

There is **no mixed mode**: a single NaN in any slot forces auto on the whole
batch. This is the explicit TODO at annotation.jl:447-449. Workaround: the
user can pre-compute desired positions and pass them all as a finite vector
(making it fully manual), or use multiple `annotation!` calls.

### 5.2 The `reset` flag

`reset` (annotation.jl:318, 424, 439) is a *callback to the algorithm* asking
it to discard any previous solution. It's set by the recipe as:

```julia
reset = !advance
```

with `advance` true iff `__advance_optimization` is the *only* changed input
(annotation.jl:300). In `Automatic` dispatch, `reset` zeros the `offsets`
array unconditionally (annotation.jl:439-441) **before** the manual / auto
branch. In `LabelRepel`, `reset` is *not* a parameter — but because the
outer dispatch has already zeroed `offsets`, the `iszero(offsets)` check
at annotation.jl:469 fires the centering pre-pass. So in practice:

* **`reset = true`** (the normal case) → start from zero offsets, run
  `maxiter` iterations of `LabelRepel`, including the one-shot centering bias.
* **`reset = false`** (advance mode) → keep current offsets, run
  `__advance_optimization` more iterations without re-biasing. Useful for
  interactive "step forward N times" debugging.

### 5.3 The "fully manual" → "fully auto" → "mixed" decision tree

| Input shape | `label_offsets_or_positions` | Outer behavior |
|-------------|------------------------------|----------------|
| `annotation(x,y)` or `annotation(p)` or `annotation(xs,ys)` or `annotation(v)` | all `NaN` | Auto: delegate to `LabelRepel` (or whatever `algorithm` is). |
| `annotation(x,y,x2,y2)` or `annotation(p1,p2)` or `annotation(v1,v2)` or 4-vector form | all finite | Manual: `offsets = positions − targets` in pixels. No iteration. |
| User constructs `Vec2d`s manually with some NaNs | mixed | Treated as **auto** (algorithm runs on all). The finite slots are passed in as `textpositions_offset[i]` but the `LabelRepel` solver does not use them. Use a custom algorithm to honor mixed input. |

---

## 6. Coordinate spaces — `labelspace = :relative_pixel` vs `:data`

`labelspace` (annotation.jl:163, default `:relative_pixel`) controls
**what the user means** by `label_offsets_or_positions[i]`. It is consumed in
exactly one place: the `screenpoints_label` compute node (annotation.jl:272-287).

### 6.1 `:relative_pixel` (default)

`label_offsets_or_positions[i]` is interpreted as a **pixel-space offset
relative to the target**. The label's final pixel anchor is
`screenpoints_target[i] + label_offsets_or_positions[i]`.

* **Camera-invariant:** moving the camera does not change the user-facing
  position interpretation. The recipe explicitly *suppresses* camera change
  propagation (annotation.jl:280: `return (nothing,)` when only camera-y
  inputs changed). This means in `:relative_pixel` mode, label-target pixel
  separation stays constant under pan/zoom — the labels follow their targets
  rigidly in screen space.
* **`Vec2d(NaN)`** propagates: `target_px + NaN = NaN`, so for auto mode the
  algorithm sees `screenpoints_label[i] = NaN`.

### 6.2 `:data`

`label_offsets_or_positions[i]` is interpreted as a **data-space position**
(absolute, not relative). The recipe runs the full transform-and-project
chain to map it to pixel space:

```julia
transformed_label_pos = apply_transform(tf, loffpos)
f32c_mat = f32_convert_matrix(f32c)
return (_project(Point2f, proj * f32c_mat * model, transformed_label_pos),)
```

* **Camera-variant:** the projected `screenpoints_label` changes on every
  camera/transform/model/float32 change, so the offsets computation re-runs.
  In auto mode, this means the algorithm re-runs on every pan/zoom — likely
  expensive for many labels.
* In the example at annotation.jl:1172-1183, `:data` mode is used so the
  label `(1, 20)` stays at the data coordinate (1, 20) even as the axis pans.

### 6.3 What changes between the two modes from the algorithm's POV

| Aspect | `:relative_pixel` | `:data` |
|--------|-------------------|---------|
| `textpositions` (param 3) | pixel (always) | pixel (always) |
| `textpositions_offset` (param 4) | pixel (target + user offset) | pixel (data positions projected to pixel) |
| `text_bbs` (param 5) | pixel | pixel |
| `bbox` (param 6) | pixel (viewport) | pixel (viewport) |
| Re-computes on pan/zoom? | **No** (camera changes are suppressed at the `screenpoints_label` node) | **Yes** (full projection chain re-runs) |
| What user "sees" | label N pixels from target, regardless of zoom | label at data coord, moves with axis |

**Key takeaway: the algorithm operates in pixel space in both cases.** The
`labelspace` parameter is passed to the algorithm but the default
`LabelRepel` ignores it. It exists for algorithms that may want to apply
different policies (e.g. lock-on-data semantics in `:data` mode).

### 6.4 Manual mode + labelspace interplay

In manual mode (`Automatic` dispatch, finite offsets):

* `:relative_pixel` + manual → `screenpoints_label = target + user_pixel_offset`,
  then `offsets = screenpoints_label - target = user_pixel_offset`. Identity.
* `:data` + manual → `screenpoints_label = project(user_data_pos)`,
  then `offsets = screenpoints_label - target` ≠ user_data_pos. The "offset"
  the inner `text!` receives is the pixel delta between projected position
  and target. Pans/zooms change the offset.

So manual `:data` mode gives you "label sits at this data point", while
manual `:relative_pixel` gives "label sits N pixels from its target".

---

## 7. Cross-cutting / other findings

### 7.1 `data_limits` & `boundingbox`

annotation.jl:585-586:

```julia
data_limits(p::Annotation) = Rect3f(Rect2f(p.target_positions[]))
boundingbox(p::Annotation, space::Symbol = :data) = apply_transform_and_model(p, data_limits(p))
```

The plot's data limits are derived **only from the target positions**, not
the label positions. This matters because in `:data` labelspace manual mode,
the labels could be drawn far outside the axis limits, but the plot won't
contribute those positions to autolimit calculation. (This is also the
genesis of the README's "Fix axis-limit blow-up: track data anchors only"
commit and the test for label clamping.)

### 7.2 Where the inner `text!` lives

The child `text!` plot (annotation.jl:238) is a separate `Plot{text}` instance
parented to the annotation. The annotation's `offsets` observable drives its
`offset` attribute through `update!(txt, offset = ...)` (annotation.jl:326)
rather than a compute-graph alias. Consequence: if you `text!`-introspect the
annotation, the offset you see is the *resolved* placement (target-relative
pixel offset).

### 7.3 The `plotlist!` for connectors

annotation.jl:356:

```julia
plotlist!(p, p.plotspecs; visible = p.visible[])
```

Every connector (line, arrowhead, optional path-text) is a `PlotSpec` in the
output list, drawn via Makie's PlotList mechanism. Each spec uses
`space = :pixel` (annotation.jl:1036, 1049, 1085, 1099) because all the
geometry has already been resolved to pixels. The arrow heads in particular
are rendered as `Scatter` of a custom `BezierPath` marker (annotation.jl:1097-1100).

### 7.4 The `_guard_nonfinite` empty-string handling

annotation.jl:233:

```julia
_guard_nonfinite(bb) = isfinite_rect(bb) ? bb : Rect2d(0, 0, 0, 0)
```

Empty-string text produces an `Rect3d()` with `Inf` widths (`isfinite_rect`
defined at data_limits.jl:55). Without this guard, `text_bbs .+ px_pos` would
produce NaN-laced rectangles and the algorithm would diverge silently.
Replacing with a zero-size rect means "this label is a point" for collision
purposes.

### 7.5 The `style = automatic` resolution

annotation.jl:1013:

```julia
annotation_style_plotspecs(::Automatic, path, p1, p2; kwargs...) = annotation_style_plotspecs(Ann.Styles.Line(), path, p1, p2; kwargs...)
```

The runtime default. This means if you pass `style = automatic`, you get the
simplest line connector — no arrows. The Path stays whatever you set (so
e.g. `path = Ann.Paths.Arc(0.3), style = automatic` gives a curved line with
no arrowheads). This is *different* from the doc-stated default — the doc
says `automatic`, which is technically `Ann.Styles.Line()` after resolution.

### 7.6 The `connection_path` paths

| Path | Definition | Output |
|------|------------|--------|
| `Ann.Paths.Line` | annotation.jl:588-595 | `BezierPath([MoveTo(p1), LineTo(p2)])` |
| `Ann.Paths.Corner` | annotation.jl:597-616 | Right-angle bend; picks horizontal-then-vertical or vertical-then-horizontal based on which axis has the larger delta. |
| `Ann.Paths.Arc` | annotation.jl:662-666 | `BezierPath([MoveTo(p1), EllipticalArc(...)])` using the arc center/radius computed from chord and height. Degrades to `Line` for `\|height\| < 1e-4`. |

The startpoint is path-shape-dependent — `Line` and `Arc` use the bbox
center; `Corner` uses the bbox edge midpoint nearest the target.

### 7.7 `shrink_path` and `clip_path_from_start`

`shrink_path` (annotation.jl:668-724) and `clip_path_from_start` (annotation.jl:845-869)
together produce the visible connector geometry by:

1. Clipping the path to start *outside* the label's bounding rect
   (`clipstart`, default the offset_bb).
2. Shrinking both ends by a pixel-radius circle (`shrink`, default `(5,7)`).

Both work in screen (pixel) space and operate on `BezierPath` commands. The
shrink loop iterates commands and applies `circle_intersection` per segment.
There are TODOs at annotation.jl:683 and annotation.jl:707 about empty
`BezierPath` not working correctly — degenerate cases return a 1-command path.

### 7.8 Auto-style fallback at the `*Arrows.Line.linewidth` level

`_auto` (annotation.jl:1067-1068) is used inside per-arrow plotspecs to let
the user override per-arrow `color`/`linewidth` while inheriting from the
annotation-level defaults. Example: `Ann.Arrows.Head(color = :red)` only
overrides the head color; the rest of the connector still uses
`color = @inherit linecolor`.

### 7.9 The `attribute_examples` block

annotation.jl:1103-1190 returns a `Dict{Symbol, Vector{Example}}` consumed by
the doc-generation pipeline. There are examples for `:shrink`, `:style`,
`:path`, and `:labelspace` — no examples for `algorithm`, `maxiter`,
`clipstart`, or any text attribute.

---

## 8. Summary of integration touchpoints for an external label-placement package

Based on the above, here are the precise hooks for plugging a custom
label-placement algorithm (e.g. `MakieTextRepel`) into `annotation!`:

### 8.1 The clean integration: override `calculate_best_offsets!`

Define a struct (e.g. `TextRepelAlgo`) and a method:

```julia
function Makie.calculate_best_offsets!(
        algo::TextRepelAlgo,
        offsets::Vector{<:Vec2},
        textpositions::Vector{<:Point2},
        textpositions_offset::Vector{<:Point2},
        text_bbs::Vector{<:Rect2},
        bbox::Rect2;
        maxiter::Union{Makie.Automatic, Int},
        reset::Bool,
        labelspace::Symbol,
    )
    ...
    return
end
```

User: `annotation!(ax, xs, ys; algorithm = TextRepelAlgo(...), text = labels)`.

### 8.2 Things the external algorithm gets for free

* Pixel-space inputs (all six positional args).
* The viewport as the clamp rectangle (passed as `bbox`).
* Automatic resize-on-input-change of `offsets`.
* The `text!` plot's offset is updated automatically when `offsets` changes.
* Camera-change suppression in `:relative_pixel` mode (no churn).
* The `advance_optimization!` API for interactive stepping.

### 8.3 Things the external algorithm has to provide

* A *full* iterative solution (no incremental contract — the recipe expects
  the offset vector to be fully placed when it returns).
* A sensible default for `maxiter == automatic`.
* A reset strategy (you can respect `reset` or always reset internally — but
  if you ignore it, `advance_optimization!` won't work).
* If you want axis-limit-aware clamping (rather than viewport-aware), you'll
  have to query `parent_scene(p).camera` or accept a custom rect somehow —
  the recipe only gives you the viewport. **The recipe does not pass axis
  limits.** This is the same constraint that drove the existing
  `MakieTextRepel.jl` axis-clamp logic.

### 8.4 Things you can't change from a custom algorithm

* The interpretation of `label_offsets_or_positions` (auto vs manual). The
  `Automatic` outer dispatch decides this before delegating, and if all
  inputs are finite it short-circuits to manual without ever calling your
  algorithm. To intercept this, you'd have to **also** override the
  `Automatic` dispatch (which is fragile).
* The fact that targets are always in `:data` space and labels move in
  pixels relative to them.
* The connector geometry — that is all in `plotspecs` (annotation.jl:328-353)
  and consumes `offsets` independently.
* The choice that `text_bbs` are computed *without* any rotation knob — the
  recipe has no `rotation` attribute for the inner `text!` (the text always
  comes out at angle 0).

### 8.5 Things to watch out for during integration

1. **Per-label NaN sentinel** — if the user passes a vector with some NaNs
   for mixed mode, the outer dispatch treats it as fully auto. To honor
   per-slot NaNs, your custom algorithm must check each
   `textpositions_offset[i]` itself and skip the optimization for finite
   slots.
2. **`reset` is a required kwarg** — must be in your method signature.
3. **`labelspace` is a required kwarg** — must be in your method signature.
4. **Self-target repulsion** — the default `LabelRepel` repels labels from
   their own targets (annotation.jl:502-509 iterates over *all* targets
   inside the inner loop). If you want different behavior, exclude `i == j`.
5. **Centering bias on zero-offset start** (annotation.jl:469-477) — runs
   only on first iteration when `iszero(offsets)`. Your algorithm has the
   freedom to do this differently.
6. **No convergence detection in `LabelRepel`** — every call runs the full
   `maxiter`. Your algorithm can shortcut if you detect zero net forces.
7. **The `__advance_optimization` channel** is shared — if your algorithm
   accepts it (via `maxiter`), the user gets free interactive stepping.
8. **Reading the `Annotation` plot's `text_bbs`, `screenpoints_target`, etc.**
   from outside the algorithm is possible (they're nodes on `p.attributes`
   and exposed as `p.text_bbs[]`, etc.), but you generally want to operate
   from inside the algorithm callback to stay synchronized with the compute
   graph.

---

## Appendix A — Full attribute table with source line references

| # | Attribute | Default expr | Source | Docstring |
|---|-----------|--------------|--------|-----------|
| 1 | `textcolor` | `automatic` | annotation.jl:99 | annotation.jl:96-98 |
| 2 | `color` | `@inherit linecolor` | annotation.jl:103 | annotation.jl:100-102 |
| 3 | `text` | `""` | annotation.jl:107 | annotation.jl:104-106 |
| 4 | `font` | `@inherit font` | annotation.jl:111 | annotation.jl:108-110 |
| 5 | `fonts` | `@inherit fonts` | annotation.jl:115 | annotation.jl:112-114 |
| 6 | `fontsize` | `@inherit fontsize` | annotation.jl:119 | annotation.jl:116-118 |
| 7 | `align` | `(:center, :center)` | annotation.jl:123 | annotation.jl:120-122 |
| 8 | `justification` | `automatic` | annotation.jl:127 | annotation.jl:124-126 |
| 9 | `lineheight` | `1.0` | annotation.jl:131 | annotation.jl:128-130 |
| 10 | `path` | `Ann.Paths.Line()` | annotation.jl:136 | annotation.jl:132-135 |
| 11 | `style` | `automatic` | annotation.jl:141 | annotation.jl:137-140 |
| 12 | `shrink` | `(5.0, 7.0)` | annotation.jl:147 | annotation.jl:142-146 |
| 13 | `clipstart` | `automatic` | annotation.jl:152 | annotation.jl:148-151 |
| 14 | `maxiter` | `automatic` | annotation.jl:156 | annotation.jl:153-155 |
| 15 | `labelspace` | `:relative_pixel` | annotation.jl:163 | annotation.jl:157-162 |
| 16 | `linewidth` | `1.0` | annotation.jl:165 | annotation.jl:164 |
| 17 | `algorithm` | `automatic` | annotation.jl:170 | annotation.jl:166-169 |
| 18 | `visible` | `true` | annotation.jl:172 | annotation.jl:171 |
| 19 (internal) | `viewport` | (linked) | annotation.jl:289 | — |
| 20 (internal) | `__advance_optimization` | `0` | annotation.jl:290 | — |
| 21 (internal) | `space` | `:data` | annotation.jl:257 | — |

Computed (not user-facing) nodes registered on `p.attributes`:

* `computed_textcolor` (annotation.jl:236)
* `screenpoints_target` (annotation.jl:258) — via `register_projected_positions!`
* `text_bbs` (annotation.jl:263)
* `world_to_pixel` (annotation.jl:267) — via `register_camera_matrix!`; also `f32c`, `model`, `transform_func` get injected here
* `screenpoints_label` (annotation.jl:272)
* `offsets` (annotation.jl:298)
* `plotspecs` (annotation.jl:332)

---

## Appendix B — Source map (line ranges)

| Range | Content |
|-------|---------|
| 1-78 | `baremodule Ann` (Paths, Arrows, Styles) |
| 80 | `using .Ann` |
| 82-94 | Function docstring for `annotation` |
| 95-173 | `@recipe Annotation (...)` block — all 18 attributes |
| 175-195 | `closest_point_on_rectangle` (unused publicly; reachable via clipstart math) |
| 197-229 | Seven `convert_arguments` overloads |
| 231-233 | `_guard_nonfinite` |
| 235-359 | `plot!(p::Annotation)` — the compute-graph wiring |
| 361-365 | `advance_optimization!` |
| 367-413 | `distance_point_outside_rect`, `distance_point_inside_rect` |
| 415-419 | `LabelRepel` struct |
| 421-452 | `calculate_best_offsets!(::Automatic, ...)` — outer dispatch |
| 454-531 | `calculate_best_offsets!(::LabelRepel, ...)` — solver |
| 533-565 | `interval_overlap`, `rect_overlap` |
| 567-583 | `startpoint(::Line, ...)`, `startpoint(::Corner, ...)` |
| 585-586 | `data_limits`, `boundingbox` |
| 588-595 | `connection_path(::Line, ...)` |
| 597-616 | `connection_path(::Corner, ...)` |
| 618-620 | `startpoint(::Arc, ...)` |
| 622-660 | `circle_centers`, `arc_center_radius` |
| 662-666 | `connection_path(::Arc, ...)` |
| 668-724 | `shrink_path` |
| 726-733 | `reversed_command` |
| 735-838 | `circle_intersection` (LineTo and EllipticalArc cases) |
| 840-843 | `is_between` |
| 845-869 | `clip_path_from_start` |
| 871-1011 | `bbox_containment`, `bbox_intersection`, `circle_line_intersection`, `line_rectangle_intersection` |
| 1013-1065 | `annotation_style_plotspecs` for `Automatic`, `LineArrow`, `Line`, `WithText` |
| 1067-1068 | `_auto` |
| 1070-1074 | `shrinksize` |
| 1076-1101 | `plotspecs` for `Arrows.Line` and `Arrows.Head` |
| 1103-1190 | `attribute_examples(::Type{Annotation})` |

---

## Appendix C — Open TODOs in the source

* annotation.jl:355: "TODO: passing dynamic attributes doesn't work (visible)"
  — `p.visible[]` is deref'd at plotlist creation; subsequent visible
  toggles won't propagate.
* annotation.jl:447-449: TODO for mixed manual/auto per-label support.
* annotation.jl:683, 707: "empty BezierPath doesn't work currently because of bbox"
  — workaround: returns a 1-command path. Affects degenerate cases where
  the entire path is inside the shrink circle.
* annotation.jl:826: "TODO: which one to pick?" — in the
  `circle_intersection` for EllipticalArc, when two intersection points
  both fall on the arc, the code arbitrarily picks the first.
* annotation.jl:877, 924: "TODO: implement" for `bbox_containment` and
  `bbox_intersection` of ellipses (not circular arcs). Currently errors
  for non-circular ellipses.
* annotation.jl:299: "We should only advance if it's the only thing causing
  an update?" — phrased as a question; the current implementation does
  enforce this, but the author noted uncertainty.
* annotation.jl:303: "Probably required when input sizes change?" — confirms
  the resize call is defensive.

---

## Appendix D — Useful selectors for future inspection

In the compute graph, the following are exposed as `p.<name>[]`:

* `p.label_offsets_or_positions[]` — raw input (manual positions or NaN sentinels)
* `p.target_positions[]` — raw target points
* `p.screenpoints_target[]` — projected targets in pixels
* `p.screenpoints_label[]` — projected/offset labels in pixels (NaN when auto)
* `p.text_bbs[]` — per-label bboxes anchored at target pixel
* `p.offsets[]` — resolved pixel offsets the algorithm produced
* `p.plotspecs[]` — the rendered connector specs
* `p.computed_textcolor[]` — resolved text color
* `p.viewport[]` — current viewport Rect2i

For a custom algorithm wanting to introspect what Makie sees on a given
plot: `Makie.calculate_best_offsets!` will receive precisely
`(p.screenpoints_target[], p.screenpoints_label[], p.text_bbs[],
Rect2d((0,0), widths(p.viewport[])))` — modulo timing, since these are all
live observables.

