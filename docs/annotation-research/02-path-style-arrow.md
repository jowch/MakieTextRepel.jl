# Annotation Rendering Machinery — Paths, Styles, Arrows, Clipping

**Scope:** `Ann.Paths`, `Ann.Styles`, `Ann.Arrows`, `connection_path`,
`startpoint`, `annotation_style_plotspecs`, `shrink_path`, `clip_path_from_start`,
and the plotspec emission pipeline in `Makie.annotation!`.

**Primary source file:**
`/home/jonathanchen/.julia/packages/Makie/p9K7f/src/basic_recipes/annotation.jl`
(1190 LOC). Supporting: `src/bezier.jl` (path primitives) and
`src/basic_recipes/pathtext.jl` (text-on-path recipe used by `WithText`).

---

## Abstract — Most Important Findings

1. **The whole "connector visual" is just `BezierPath` + a `PlotSpec` list.**
   Every annotation produces 1 `Lines` plot (the path) plus optional head/tail
   scatter or line plots, and optionally one `PathText` plot. There is no
   bespoke renderer — it lives entirely in `annotation_style_plotspecs` and
   delegates to existing recipes via `PlotSpec`. This makes the visual surface
   pleasantly small to integrate with.

2. **Three path types, no extension API.** `Ann.Paths.Line`, `Ann.Paths.Corner`,
   and `Ann.Paths.Arc(height)` are the entire set. The dispatch happens through
   `connection_path(::Ann.Paths.X, p1, p2)` (`annotation.jl:588, 597, 662`)
   plus `startpoint(::Ann.Paths.X, text_bb, p2)` (`:567, 569, 618`). A user
   can technically add a method, but the modules are `baremodule`s with no
   exports and the dispatch is undocumented — treat it as internal.

3. **`p2 in offset_bb && return` is the kill-switch.** When the data anchor
   point lies inside the (offset) text bounding box, the *entire* PlotSpec
   for that label is skipped — no line, no arrow, no PathText
   (`annotation.jl:337`). This is the single most important behavior for
   anyone driving label placement: a tight label sitting on its anchor
   silently has no connector. There is no user knob to disable this.

4. **`shrink = (5.0, 7.0)` is a per-end circular clip in pixel space.**
   `shrink[1]` (default 5px) is the radius around the path's start
   (`startpoint`, typically the text bbox center); `shrink[2]` (default 7px)
   is the radius around the data point. Both are evaluated by
   `shrink_path` (`:668`) and run *after* the bbox clip. To disable, set
   `shrink = (0.0, 0.0)`.

5. **There is no `clipend` attribute** — only `clipstart`. The data-anchor
   end is shaped exclusively by `shrink[2]`. `clipstart` defaults to the
   text bbox; users can substitute any `Rect2`.

6. **`LineArrow` re-shrinks the path internally** to leave room for the
   arrowhead/tail, on top of the global `shrink` pass. This double-shrink
   means tight labels can produce empty paths and a single arrow head
   floating at `p2` (`annotation.jl:1015–1045`).

7. **`Arc.height` is *fractional* relative to the chord length.** `height = 1`
   is a half-circle; `height < 1e-4` falls back to a straight line. Sign
   controls the bulge direction (`annotation.jl:662–666`).

---

## 1. The `Ann` Module Tree (annotation.jl:1–78)

The whole namespace is built as a `baremodule` tree "for cleanest
tab-completion behavior" (the literal comment at line 1). The structure:

```julia
baremodule Ann
    baremodule Paths     # Line, Corner, Arc
    baremodule Arrows    # Line, Head
    baremodule Styles    # Line, LineArrow, WithText
end
using .Ann
```

`Ann` is exported from `Makie.jl:435` (`export Ann`). The three sub-modules
are NOT exported — users always write `Ann.Paths.X`, `Ann.Styles.X`,
`Ann.Arrows.X`. None of the types inside them are exported.

`baremodule` means: no implicit `using Base`, no implicit `eval`, no implicit
`include`. The author re-introduces `using Base` and selectively imports
`Makie`/`Arrows` so the leaf modules see only what they need. The
side-effect for us: the modules are *intended* to look like static "enums of
variants," not extensible registries.

---

## 2. `Ann.Paths` — Path Type Catalog

Defined at `annotation.jl:4–14`:

```julia
baremodule Paths
    using Base
    struct Line end
    struct Corner end
    Base.@kwdef struct Arc
        height::Float64 = 0.5  # >0 up-then-down, <0 down-then-up, 1 is half-circle
    end
end
```

That's it. Three types. Two singleton structs and one parametric struct
with one field. All three appear as the `path` attribute of the
`@recipe Annotation`, default `Ann.Paths.Line()` (`:136`).

A path type is consumed by *exactly two* dispatch hooks:

1. `startpoint(path, text_bb, p2) -> Point2d` — where on the text bounding
   box does the connector emerge from.
2. `connection_path(path, p1, p2) -> BezierPath` — given that start point
   and the data anchor, build the geometry.

That's the entire path-type contract.

### 2.1 `Ann.Paths.Line`

- **Start point** (`:567`): geometric **center of the text bbox**.

  ```julia
  startpoint(::Ann.Paths.Line, text_bb, p2) = text_bb.origin + 0.5 * text_bb.widths
  ```

- **Path** (`:588`): single straight segment.

  ```julia
  function connection_path(::Ann.Paths.Line, p1, p2)
      return BezierPath([MoveTo(p1), LineTo(p2)])
  end
  ```

- **Trace** for `text_bb = Rect2d((100, 100), (40, 20))`, `p2 = (300, 250)`:
  - `p1 = (120, 110)` (center of bbox)
  - `BezierPath([MoveTo((120,110)), LineTo((300,250))])`

  The straight line will then have its `p1` end clipped to the bbox edge by
  `clip_path_from_start` (because `p1` is *inside* the bbox), and both ends
  shrunk by 5/7px circles. Net effect: a clean line from bbox edge to
  ~7px outside the anchor.

### 2.2 `Ann.Paths.Corner`

- **Start point** (`:569–583`): edge of the bbox **on the side closest to
  `p2`**. Compares `|dir_x|` vs `|dir_y|`; emits at right/left midpoint or
  top/bottom midpoint accordingly.

  ```julia
  function startpoint(::Ann.Paths.Corner, text_bb, p2)
      l, r, b, t = left, right, bottom, top of bbox
      dir = p2 - (text_bb.origin + 0.5 * text_bb.widths)
      if abs(dir[1]) < abs(dir[2])   # vertical separation dominates
          x = dir[1] > 0 ? r : l
          y = (t + b) / 2
      else
          x = (l + r) / 2
          y = dir[2] > 0 ? t : b
      end
      return Point2d(x, y)
  end
  ```

  Note the comparison is `abs(dir[1]) < abs(dir[2])`, which intentionally
  picks the *less dominant* axis as the exit. Read together with the path
  builder, this produces an L-shape that "goes out the side, then turns".

- **Path** (`:597–616`): a 3-point polyline forming an L. The kink axis is
  chosen by which component of `(p2 - p1)` is larger:

  ```julia
  function connection_path(::Ann.Paths.Corner, p1, p2)
      dir = p2 - p1
      return if abs(dir[1]) > abs(dir[2])
          BezierPath([MoveTo(p1), LineTo(p1[1], p2[2]), LineTo(p2)])
      else
          BezierPath([MoveTo(p1), LineTo(p2[1], p1[2]), LineTo(p2)])
      end
  end
  ```

  So if `p2` is mostly to the right, the path first goes **up/down** to
  `p2`'s y, then **horizontally** to `p2`. If `p2` is mostly above, the
  path first goes **horizontally** to `p2`'s x, then **vertically** to
  `p2`. Always 2 segments, axis-aligned.

- **Trace** for `p1 = (140, 120)` (right-middle of bbox), `p2 = (300, 250)`:
  - `dir = (160, 130)`, `|160| > |130|`, so use first branch:
  - `BezierPath([MoveTo((140,120)), LineTo((140, 250)), LineTo((300, 250))])`

  Wait — that goes *up first then right*. Inspect again: first branch's
  middle vertex is `(p1[1], p2[2])` = `(140, 250)`. Yes, vertical leg first.
  This is a `┐` shape (going from bbox up then over to anchor). The example
  in attribute_examples (`:1158`) uses `Corner()` to land on a point
  below-right, producing exactly this routing.

### 2.3 `Ann.Paths.Arc(height)`

- **Start point** (`:618`):

  ```julia
  startpoint(::Ann.Paths.Arc, text_bb, p2) = center(text_bb)
  ```

  Same as `Line`. The arc bulges outward from the chord
  (text-bbox-center → p2).

- **Geometry helpers** (`:641–660`):

  ```julia
  function arc_center_radius(p1, p2, x)
      xabs = abs(x)
      chord = p2 - p1
      mid = midpoint(p1, p2)
      len = norm(chord)
      height = xabs * len / 2           # *** half-chord-relative ***
      r = (len^2) / (8height) + height / 2
      perp = normalize(Point2(-chord[2], chord[1]))
      direction = sign(x) * chord[1] > 0 ? -1 : 1
      center = mid + direction * perp * (r - height)
      return r, center
  end
  ```

  Key insight: `height = xabs * len / 2`. So the user's `Arc(height=0.5)`
  produces an actual sagitta equal to `0.5 * chord_length / 2 = chord/4`.
  And `Arc(height=1)` produces sagitta = `chord/2`, which is by definition
  a half-circle (radius = chord/2). The classical relation
  `r = (len² / 8h) + h/2` then drops out cleanly.

- **Path** (`:662–666`):

  ```julia
  function connection_path(ca::Ann.Paths.Arc, p1, p2)
      abs(ca.height) < 1.0e-4 && return connection_path(Ann.Paths.Line(), p1, p2)
      radius, center = arc_center_radius(p1, p2, ca.height)
      return BezierPath([
          MoveTo(p1),
          EllipticalArc(center, radius, radius, 0.0,
                        atan(reverse(p1 - center)...),
                        atan(reverse(p2 - center)...)),
      ])
  end
  ```

  Notes:
  - `r1 == r2 == radius`, so it's actually a **circular** arc despite
    using `EllipticalArc`. The angle parameter is `0`.
  - `atan(reverse(p - center)...)` is `atan(dy, dx)` — the
    `reverse(...)` swaps the components since the tuple is `(dx, dy)`.
  - The fallback to `Line` for very small heights (`< 1e-4`) avoids
    blowups in `arc_center_radius` (which errors out when `height == 0`,
    `:647–649`).

- **Trace** for `p1 = (100, 200)`, `p2 = (300, 200)`, `height = 0.5`:
  - `chord = (200, 0)`, `len = 200`
  - `height_pixel = 0.5 * 200 / 2 = 50`
  - `r = 200² / (8·50) + 50/2 = 100 + 25 = 125`
  - `perp = normalize((0, 200)) = (0, 1)`
  - `sign(0.5) * 200 > 0` ⇒ `direction = -1`
  - `center = (200, 100) + (-1)·(0,1)·(125-50) = (200, 25)`
  - Start angle: `atan(200-25, 100-200) = atan(175, -100) ≈ 2.09 rad (120°)`
  - End angle:   `atan(200-25, 300-200) = atan(175,  100) ≈ 1.05 rad (60°)`
  - Resulting `EllipticalArc` sweeps from 120° to 60° (clockwise in
    Makie's convention because `a2 < a1`), centered at (200, 25),
    radius 125. The top of the arc reaches y = 25+125 = 150 — below
    the chord, **i.e. the arc bulges downward**. The "direction" sign
    line means positive `height` curves opposite to what one might guess
    from "up" without considering the chord orientation; in practice
    users iterate the sign to get the curve they want.

### 2.4 `data_limits` for Annotation

`data_limits(p) = Rect3f(Rect2f(p.target_positions[]))` (`:585`). Only
the target anchors contribute to data limits. **The connector geometry
itself does not affect axis bounds** — this is by design (and matches
the post-rebase MakieTextRepel behavior we already noticed). The path
lives in `:pixel` space anyway (see plotspecs below).

---

## 3. `Ann.Arrows` — Arrowhead & Tail Catalog

Defined at `annotation.jl:16–35`:

```julia
baremodule Arrows
    Base.@kwdef struct Line
        length::Float64 = 8.0
        angle::Float64 = deg2rad(60)
        color = Makie.automatic
        linewidth::Union{Makie.Automatic, Float64} = Makie.automatic
    end
    Base.@kwdef struct Head
        length::Float64 = 8.0
        angle::Float64 = deg2rad(60)
        color = Makie.automatic
        notch::Float64 = 0  # 0 to 1
    end
end
```

Two types, both `kwdef`.

### 3.1 `Ann.Arrows.Line` — Open V

Rendered via `plotspecs(::Ann.Arrows.Line, pos; rotation, color, linewidth)`
at `:1076–1087`:

```julia
function plotspecs(l::Ann.Arrows.Line, pos; rotation, color, linewidth)
    color = _auto(l.color, color)
    linewidth = _auto(l.linewidth, linewidth)
    sidelen = l.length / cos(l.angle / 2)
    dir1 = Point2(-cos(l.angle / 2 + rotation), -sin(l.angle / 2 + rotation))
    dir2 = Point2(-cos(-l.angle / 2 + rotation), -sin(-l.angle / 2 + rotation))
    p1 = pos + dir1 * sidelen
    p2 = pos + dir2 * sidelen
    return [PlotSpec(:Lines, [p1, pos, p2]; space = :pixel, color, linewidth)]
end
```

- Emits a **single `Lines` plotspec** with 3 vertices: `p1`–`pos`–`p2`.
- This is the "open" arrowhead — two strokes meeting at the tip.
- `length` is the distance from tip *along the centerline*; `sidelen` is
  the longer side length after compensating for half-angle.
- The `_auto(...)` calls let per-arrow `color`/`linewidth` override the
  inherited annotation defaults.

### 3.2 `Ann.Arrows.Head` — Filled Triangle (with optional notch)

Rendered via `plotspecs(::Ann.Arrows.Head, pos; ...)` at `:1089–1101`:

```julia
function plotspecs(h::Ann.Arrows.Head, pos; rotation, color, linewidth)
    color = _auto(h.color, color)
    len = h.length
    L = 1 / cos(h.angle / 2)
    p1 = L * Point2(-cos(h.angle / 2), -sin(h.angle / 2))
    p2 = Point2(-(1 - h.notch), 0)
    p3 = L * Point2(-cos(-h.angle / 2), -sin(-h.angle / 2))

    marker = BezierPath([MoveTo(0,0), LineTo(p1), LineTo(p2), LineTo(p3), ClosePath()])
    return [PlotSpec(:Scatter, pos;
                     space = :pixel, rotation, color, marker, markersize = len)]
end
```

- Emits a **single `Scatter` plotspec** at `pos`, using a custom
  `BezierPath` marker — a closed quadrilateral
  `(0,0) → p1 → p2 → p3 → close`.
- `notch ∈ [0, 1]` carves a wedge into the back of the head: `notch = 0`
  ⇒ flat back at x=-1; `notch = 1` ⇒ back collapses to the tip
  (effectively two open lines, but as a filled shape).
- The rotation is delegated to `Scatter`'s `rotation` attribute.
- Note: `linewidth` is *not* used for the filled triangle (it's a fill).

### 3.3 Shrink budget per arrow

`shrinksize` (`:1070–1074`) controls how much the path is shortened to
leave room for the arrowhead:

```julia
shrinksize(other) = 0.0
shrinksize(l::Ann.Arrows.Head) = l.length * (1 - l.notch)
```

So `Ann.Arrows.Line` consumes **zero** path length (the V-arrow is drawn
overlapping the endpoint), while `Ann.Arrows.Head` reserves
`length * (1 - notch)` pixels. `nothing` (no arrow) also yields `0.0`.

---

## 4. `Ann.Styles` — Style Catalog

Defined at `annotation.jl:37–77`:

```julia
baremodule Styles
    using ..Arrows: Arrows
    using ...Makie: Makie

    struct Line end                            # plain line, no arrows

    Base.@kwdef struct LineArrow
        head = Arrows.Line()                   # arrow at the data-anchor end
        tail = nothing                         # optional arrow at the label end
    end

    struct WithText                            # decorator wrapping any other style
        style::Any
        text::Any
        fontsize::Float64
        align::Any
        offset::Float64
        color::Any
    end
    function WithText(style; text="", fontsize=12.0,
                      align=(:center, :bottom), offset=4.0,
                      color=Makie.automatic)
        return WithText(style, text, Float64(fontsize), align, Float64(offset), color)
    end
end
```

Three style types. Each one implements
`annotation_style_plotspecs(style, path, p1, p2; color, linewidth) -> Vector{PlotSpec}`.

### 4.1 Style dispatch entry point

The default style is `automatic`, dispatched at `:1013`:

```julia
annotation_style_plotspecs(::Automatic, path, p1, p2; kwargs...) =
    annotation_style_plotspecs(Ann.Styles.Line(), path, p1, p2; kwargs...)
```

So `automatic` ≡ `Ann.Styles.Line()`.

### 4.2 `Ann.Styles.Line` — Just a line

`annotation.jl:1047–1051`:

```julia
function annotation_style_plotspecs(::Ann.Styles.Line, path, p1, p2; color, linewidth)
    return [PlotSpec(:Lines, path; color, linewidth, space = :pixel)]
end
```

A single `:Lines` plotspec consuming the `BezierPath` directly. No arrow,
no head, no tail. This is what gets drawn when `style = Ann.Styles.Line()`.

### 4.3 `Ann.Styles.LineArrow` — Line + head + optional tail

`annotation.jl:1015–1045`:

```julia
function annotation_style_plotspecs(l::Ann.Styles.LineArrow, path, p1, p2; color, linewidth)
    length(path.commands) < 2 && return PlotSpec[]      # already-empty path → nothing
    p_head = endpoint(path.commands[end])
    p_tail = path.commands[1].p                          # MoveTo's point

    shrink_for_head = shrinksize(l.head)                 # 0 or len*(1-notch)
    shrink_for_tail = shrinksize(l.tail)                 # always 0 currently

    shortened_path = shrink_path(path, (shrink_for_tail, shrink_for_head))
    length(shortened_path.commands) < 2 && return PlotSpec[]

    head_dir = normalize(p2 - endpoint(shortened_path.commands[end]))
    head_rotation = atan(head_dir[2], head_dir[1])
    tail_dir = normalize(p1 - shortened_path.commands[1].p)
    tail_rotation = atan(tail_dir[2], tail_dir[1])

    specs = [PlotSpec(:Lines, shortened_path; color, space = :pixel, linewidth)]
    if l.head !== nothing
        append!(specs, plotspecs(l.head, p_head; rotation = head_rotation, color, linewidth))
    end
    if l.tail !== nothing
        append!(specs, plotspecs(l.tail, p_tail; rotation = tail_rotation, color, linewidth))
    end
    return specs
end
```

Important subtleties:

1. **Pre-shrunk path is shrunk *again*** inside `LineArrow`. The first
   shrink happens in `plot!` with `(shrink[1], shrink[2])`; here it
   happens with `(tail_shrink, head_shrink)`. If you stack arrowheads
   tight against tight shrink values, you'll get empty `shortened_path`s
   and **no line at all** — only the head/tail markers remain
   (`:1027` early-return path renders just the heads).
2. **Empty input path → empty output.** If the *original* path coming in
   already has `< 2` commands (only the `MoveTo` left after global
   shrink), this style emits `PlotSpec[]` (line 1016). The head/tail
   then never get drawn — the whole annotation disappears visually.
3. **Rotation is computed from the *shortened* path's last segment**
   pointing toward `p2`, not from the path tangent at its endpoint.
   That means for very short arrows the rotation can be unstable when
   the shortened path is almost a single point.

### 4.4 `Ann.Styles.WithText` — Decorator that layers `PathText`

`annotation.jl:1053–1065`:

```julia
function annotation_style_plotspecs(s::Ann.Styles.WithText, path, p1, p2; color, linewidth)
    specs = annotation_style_plotspecs(s.style, path, p1, p2; color, linewidth)
    textcolor = s.color === automatic ? color : s.color
    push!(specs, PlotSpec(:PathText, path;
                          text = s.text, fontsize = s.fontsize,
                          align = s.align, offset = s.offset,
                          color = textcolor, space = :pixel))
    return specs
end
```

- Recursively defers to `s.style` to draw the underlying connector
  (so you can write `WithText(LineArrow())`).
- Appends *one* `:PathText` plotspec following the same path (i.e. the
  text follows the curve, even arcs and corners).
- `align` defaults to `(:center, :bottom)` — text floats above the path.
- `offset = 4.0` adds 4px perpendicular offset.
- The `PathText` recipe (`pathtext.jl:1–36`) supports `halign` as a
  fraction along the arclength, and `valign` symbolic; `WithText`
  inherits this through the `align` field which is passed as-is.

### 4.5 Style composability

The decorator pattern means `WithText` can wrap anything that has an
`annotation_style_plotspecs` method, including itself. There's no
explicit cycle check, but realistically you'd wrap `Line`/`LineArrow`
and call it a day. For a label-placement integration we get *almost*
the entire visual surface by emitting our own list of `PlotSpec`s that
imitate `annotation_style_plotspecs`.

---

## 5. The Emission Pipeline — From Attributes to PlotSpecs

The relevant chunk is `annotation.jl:328–356` (a `map!` over a tuple of
inputs that produces a `Vector{PlotSpec}`):

```julia
inputs = [:text_bbs, :screenpoints_target, :offsets, :path, :clipstart,
          :shrink, :style, :color, :linewidth]
map!(p, inputs, :plotspecs) do text_bbs, points, offsets, path, clipstart, shrink, style, color, linewidth
    specs = PlotSpec[]
    broadcast_foreach(text_bbs, points, clipstart, offsets) do text_bb, p2, clipstart, offset
        offset_bb = text_bb + offset

        p2 in offset_bb && return                              # *** skip if anchor inside label ***
        p1 = startpoint(path, offset_bb, p2)
        _path = connection_path(path, p1, p2)

        clipstart = clipstart === automatic ? offset_bb : clipstart
        clipped_path = clip_path_from_start(_path, clipstart)
        shrunk_path  = shrink_path(clipped_path, shrink)

        append!(specs, annotation_style_plotspecs(style, shrunk_path, p1, p2;
                                                  color, linewidth))
    end
    return specs
end
plotlist!(p, p.plotspecs; visible = p.visible[])
```

The flow per label:

1. **Compute `offset_bb`** = `text_bb + offset`. `text_bb` is the
   text bbox in pixel space *before* the offset (from
   `register_raw_string_boundingboxes!` at line 255); `offset` is the
   placement-algorithm-computed displacement.
2. **Anchor-in-bbox guard** (`:337`): `p2 in offset_bb && return` — see §6.
3. **Compute `p1`** via `startpoint(path, offset_bb, p2)`.
4. **Build the raw `BezierPath`** via `connection_path(path, p1, p2)`.
5. **Clip the start against `clipstart`** (`:341–346`). If `clipstart`
   is `automatic`, the offset bbox itself is the clip region.
6. **Shrink both ends by circles** (`:348`): `shrink_path(path, shrink)`.
7. **Emit style-specific plotspecs** via
   `annotation_style_plotspecs(style, shrunk_path, p1, p2; color, linewidth)`.

The final `plotlist!` materializes the spec list as Makie child plots.

---

## 6. The "Anchor In Bbox" Skip — `p2 in offset_bb && return`

(`annotation.jl:337`)

**What it does:** If the user's data point `p2`, projected into pixel
space, lies inside the (offset-translated) text bounding box, the entire
emission step for that label is silently skipped — **no line, no arrow,
no PathText, nothing.**

**Why it exists:** A connector that originates from the bbox edge and
ends *inside* the bbox would either (a) produce a zero-length path
after `clip_path_from_start` clips it to the bbox boundary, or
(b) be a visually meaningless squiggle pointing back into the label.
Both are worse than no connector.

**When it triggers:** in practice, whenever the label is placed *over*
its own data point. The most common cases:

- A small label that just got pushed to the data point by the
  repulsion solver (`LabelRepel`'s attractive force).
- `labelspace = :relative_pixel` with a small `offset = (0,0)` and a
  long label string that engulfs the anchor.
- Manual placement that puts a label box around its point.

**What controls the threshold:** the offset bbox boundary. There is
*no scalar threshold* — it's purely "inside or not". Knobs that
influence it:

- `text` (changes bbox width/height).
- `align` (changes bbox origin relative to anchor).
- `offset` (shifts the bbox).
- `padding` in the `LabelRepel` algorithm (`:418`) adds 6×5 px around
  the bbox during placement, but is *not* applied here in the
  emission step — the raw `text_bbs + offsets` is what gets tested.

**No way to disable.** The check is hardcoded; a user cannot opt out.
For an external label-placement package (e.g. MakieTextRepel) this
means: if your algorithm puts a label over its own point, the
connector quietly vanishes. The fix is to ensure offsets push the label
out of the bbox first.

**Other "empty path" failure modes:**

- `shrink_path` returns `BezierPath(path.commands[1:1])` when the path
  is fully inside one of the shrink circles (`:683`, `:707`). That's a
  1-command (MoveTo-only) path. `LineArrow` then early-returns
  `PlotSpec[]` (`:1027`), and `Line` emits a degenerate `:Lines` spec
  that draws nothing.
- `clip_path_from_start` may return the original path unmodified if no
  segment intersects `clipstart` (and no segment is contained); in that
  case the start may still poke into the bbox a tiny bit, but Makie
  doesn't visibly suffer.

---

## 7. Clipping & Shrinking — Mechanics in Detail

### 7.1 `shrink = (5.0, 7.0)` defaults

Defined at `:147`. Two-tuple `(start_radius, end_radius)` in pixels.

Quoting the docstring: *"each number specifies the radius of a circle
in screen space which clips the connection path at the start or end,
respectively, to add a little bit of visual space between arrow and
label or target."*

- `shrink[1]` is the **start** circle, centered at `p1` (the
  `startpoint` from §2). It clips the segment leaving the label.
- `shrink[2]` is the **end** circle, centered at the *original* path's
  endpoint (the data anchor `p2`). It clips the segment entering the
  target.

### 7.2 `shrink_path` algorithm (`annotation.jl:668–724`)

Pseudocode:

```
function shrink_path(path, (s1, s2)):
    if len(commands) < 2: return path
    if s1 > 0:
        walk commands forward from i=2:
            does circle_intersection(p1, s1, prev_end, command) hit?
                no → if last command: return MoveTo-only path
                no → continue (command is entirely inside circle)
                yes → replace path[1:i] with [new_moveto; new_command]; break
    if s2 > 0:
        walk commands backward from i=end:
            reverse the command, test circle_intersection against (stop, s2)
            no/contained → if i==2: return MoveTo-only path
            yes → replace path[i:end] with reversed-and-clipped command; break
    return path
```

`circle_intersection` is implemented for `LineTo` (`:735–785`) and for
**circular** `EllipticalArc` (`:787–838`, with a guard
`error("Not implemented for ellipses")` when `r1 != r2`).

The "completely contained" branch (`:683`, `:707`) is the silent
empty-path case. Comment says: *"empty BezierPath doesn't work
currently because of bbox"*, so a MoveTo-only path is the de-facto
sentinel for "render nothing".

### 7.3 `clipstart` (`:152`, default `automatic`)

Logic at `:341–346`:

```julia
clipstart = if clipstart === automatic
    offset_bb
else
    clipstart
end
clipped_path = clip_path_from_start(_path, clipstart)
```

User can pass any `Rect2` (or array thereof for broadcasting). Typical
use cases:

- Manually override with a larger rect to clip more aggressively
  (e.g. include the label's padding).
- Pass a hand-picked rect to clip the line into a non-label visual.

`clip_path_from_start` (`:845–869`) walks commands forward, drops those
fully contained in the bbox, and where the next command leaves the
bbox, replaces it with `[MoveTo(intersection_point), partial_command]`.
`bbox_containment` and `bbox_intersection` are implemented for `LineTo`
and circular `EllipticalArc`; ellipses error out.

### 7.4 No `clipend` attribute

**There is no `clipend`.** Searching `clipend` in the source returns
zero hits. The end of the path is shaped *only* by `shrink[2]`. If a
user wants to clip to the data-side scatter marker bbox, they'd have
to bake it into `shrink[2]` as a numeric radius. This is a meaningful
limitation for integration: if MakieTextRepel wants to honor a custom
marker shape on the data side, it can't piggyback on annotation's
clipping.

### 7.5 Disabling clipping

- **Disable start shrink:** `shrink = (0.0, 7.0)`.
- **Disable end shrink:** `shrink = (5.0, 0.0)`.
- **Disable both:** `shrink = (0.0, 0.0)`.
- **Disable start bbox clip:** pass `clipstart = Rect2f(0, 0, 0, 0)`
  (zero-size rect at origin), which `clip_path_from_start` will treat
  as a degenerate empty region. (`p2 in offset_bb` short-circuit still
  applies, but the actual `clipstart` rect can be overridden away from
  the text bbox.)

---

## 8. Extension Surface — Could a User Define Their Own Path/Style/Arrow?

### 8.1 What's needed

For a custom **Path** type `MyPath`, you'd need two methods *in `Makie`'s
namespace*:

```julia
Makie.startpoint(::MyPath, text_bb, p2) = ...   # returns Point2d
Makie.connection_path(::MyPath, p1, p2) = ...   # returns BezierPath
```

That's literally the entire contract. The type itself can live anywhere.

For a custom **Style** type `MyStyle`, you'd need one method:

```julia
Makie.annotation_style_plotspecs(s::MyStyle, path::BezierPath, p1, p2; color, linewidth) =
    PlotSpec[...]
```

For a custom **Arrow** type, two methods:

```julia
Makie.plotspecs(a::MyArrow, pos; rotation, color, linewidth) = PlotSpec[...]
Makie.shrinksize(a::MyArrow) = pixels::Float64
```

### 8.2 Will it work?

Mechanically, yes — Julia's multiple dispatch doesn't care that the
modules are `baremodule`s. The functions `startpoint`,
`connection_path`, `annotation_style_plotspecs`, `plotspecs`, and
`shrinksize` are all defined in `Main.Makie` (not inside the
`baremodule`s). They're just functions of the type tag.

However:

- **None of these functions are exported** from `Makie`. Users have to
  reach for `Makie.startpoint`, `Makie.connection_path`, etc.
- **No docstrings document the extension contract** — the patterns
  must be reverse-engineered from the source.
- **Path command surface is limited.** `BezierPath` supports `MoveTo`,
  `LineTo`, `CurveTo`, `EllipticalArc`, `ClosePath`. CurveTo isn't used
  by any of the built-in paths but is supported by `bbox` and the
  renderers (via `bezier.jl:96`).
- **Shrink/clip don't generalize.** `circle_intersection` and
  `bbox_intersection` are only implemented for `LineTo` and circular
  `EllipticalArc`. A custom path that uses `CurveTo` will:
  - hit `bbox_containment(::Rect2, ::Point2, ::CurveTo)` — undefined
    (it'll fall through to the LineTo/EllipticalArc methods? actually
    no, there's no fallback — it'll `MethodError`).
  - same for `circle_intersection(::Point2, r, ::Point2, ::CurveTo)`.

So a `CurveTo`-using custom path needs **the user to also add those
helper methods**, otherwise `shrink_path` and `clip_path_from_start`
will throw. The threshold for a "true" custom path is higher than it
looks.

### 8.3 Recommendation for MakieTextRepel

Treat the path/style/arrow modules as **internal/closed**. Don't ship
custom subtypes for users to extend; instead, replicate the
`annotation_style_plotspecs` pattern in our own recipe (we already do
for connectors). If we want to support arcs/corners, we should fork
the formulas inline (they're ~30 LOC total) rather than depend on
`Ann.Paths.Arc` being stable across Makie versions.

---

## 9. Cross-Reference Cheatsheet

| Symbol | Definition | Use site |
|---|---|---|
| `Ann.Paths.Line` | `:8` | `startpoint:567`, `connection_path:588` |
| `Ann.Paths.Corner` | `:9` | `startpoint:569`, `connection_path:597` |
| `Ann.Paths.Arc` | `:10–12` | `startpoint:618`, `connection_path:662` |
| `Ann.Arrows.Line` | `:22–27` | `plotspecs:1076` |
| `Ann.Arrows.Head` | `:29–34` | `plotspecs:1089`, `shrinksize:1072` |
| `Ann.Styles.Line` | `:44` | `annotation_style_plotspecs:1047` |
| `Ann.Styles.LineArrow` | `:46–49` | `annotation_style_plotspecs:1015` |
| `Ann.Styles.WithText` | `:58–75` | `annotation_style_plotspecs:1053` |
| `startpoint` | `:567, :569, :618` | called at `:338` |
| `connection_path` | `:588, :597, :662` | called at `:339` |
| `clip_path_from_start` | `:845` | called at `:346` |
| `shrink_path` | `:668` | called at `:348` and inside `LineArrow`'s emit `:1026` |
| `shrinksize` | `:1070, :1072` | called at `:1023, :1024` |
| `circle_intersection` | `:735, :787` | inside `shrink_path` |
| `bbox_intersection` | `:879, :888` | inside `clip_path_from_start` |
| `arc_center_radius` | `:641` | inside `Arc`'s `connection_path` |
| `LabelRepel` | `:415` | `calculate_best_offsets!` algorithm impl |
| `BezierPath` | `bezier.jl:196` | path container |
| `EllipticalArc` | `bezier.jl:74` | used by `Arc` |
| `PathText` recipe | `pathtext.jl:15` | used by `WithText` |

---

## 10. Concrete Plotspec Traces

### 10.1 Default `annotation!` ("Line + Line")

User: `annotation!(ax, -200, 0, 0, 0)` (one label, default everything).
After all clipping/shrinking:

```
[
  PlotSpec(:Lines, BezierPath([MoveTo(text_bb_edge_intersection), LineTo(near_p2)]);
           color, linewidth, space=:pixel)
]
```

One spec. Drawn as a simple line.

### 10.2 `style = Ann.Styles.LineArrow()` (default Line() arrow, no tail)

```
[
  PlotSpec(:Lines, shortened_path;  color, space=:pixel, linewidth),
  PlotSpec(:Lines, [p1', pos, p2']; space=:pixel, color, linewidth),  # the V
]
```

Two specs.

### 10.3 `style = LineArrow(head = Ann.Arrows.Head())`

```
[
  PlotSpec(:Lines, shortened_path; color, space=:pixel, linewidth),
  PlotSpec(:Scatter, p_head; space=:pixel, rotation, color, marker=triangle, markersize=8),
]
```

Two specs.

### 10.4 `style = LineArrow(head=..., tail=...)`

```
[
  PlotSpec(:Lines, shortened_path; color, space=:pixel, linewidth),
  PlotSpec(:Lines, [...]; ...),    # tail V
  PlotSpec(:Lines, [...]; ...),    # head V  (or :Scatter if Head)
]
```

Three specs.

### 10.5 `style = WithText(LineArrow(), text="...")`

```
[
  PlotSpec(:Lines, shortened_path; ...),
  PlotSpec(:Scatter, p_head; ..., marker=triangle),       # if head=Head
  PlotSpec(:PathText, path; text="...", fontsize=12,
           align=(:center,:bottom), offset=4, color, space=:pixel)
]
```

Three specs (more if tail is set).

---

## 11. Implications for an Integrated Label-Placement Package

(Drawing the integration-relevant conclusions explicitly, since this
is the deliverable's purpose.)

1. **We can render the same connector visuals by emitting our own
   `PlotSpec` list.** No need to depend on `Ann.Styles` — we can copy
   the ~10-line `annotation_style_plotspecs(::Line, ...)` and the
   ~30-line `LineArrow` impl into our recipe and we're done.

2. **We get the `:PathText` recipe for free.** It accepts a `BezierPath`
   directly and supports the same `(halign, valign)` semantics.

3. **The "anchor-in-bbox skip" is the gotcha to document.** Any user
   whose algorithm settles a label on its own data point will see a
   missing connector. We should either (a) move the label out by a
   pixel before emitting, (b) suppress this check by re-implementing
   the emission in our recipe, or (c) document loudly.

4. **`shrink` is the only knob for the data-side gap.** If our
   integration uses a scatter marker of varying size, the user must
   pass `shrink = (5, markersize/2 + padding)` per-label.

5. **`clipstart = automatic` is exactly the bbox of the *placed* label.**
   If our placement reports its final bbox, we can pass it as
   `clipstart` and get correct edge clipping. Otherwise the default
   uses the text bbox + offset, which is usually what we want.

6. **For arcs we should hand-roll the geometry**, copying lines
   `:641–666` into our recipe rather than depending on `Ann.Paths.Arc`.
   These formulas are unlikely to change but the type is internal.

7. **Watch out for `circle_intersection` being limited to circular arcs
   and lines.** If we add `CurveTo`-based paths we lose `shrink_path`'s
   support — we'd need to either approximate as polylines or add
   intersection routines.

---

## 12. Open Questions / Things We Didn't Cover

- **How `PathText` evaluates text along a `CurveTo`** — not relevant for
  the three built-in path types (none use `CurveTo`), but would matter
  if we add curve-based connectors. (`pathtext.jl:78–95` references
  Gauss-Legendre quadrature and inverse arc-length search from `kurbo`.)
- **Backend differences** (`CairoMakie` vs `GLMakie`) for `BezierPath`
  rendering — both accept the path directly; Cairo uses native vector
  output, GL tessellates. Probably equivalent for our purposes.
- **`visible` attribute** — the recipe comment at `:355` says
  *"TODO: passing dynamic attributes doesn't work (visible)"*. So
  toggling visibility at runtime may not propagate.
- **`maxiter` for label placement** — handled in `LabelRepel`
  (`:454–531`), not in the rendering path; out of scope here.

---

*End of report.*
