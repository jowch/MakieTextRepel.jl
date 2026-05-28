# v0.2 Design — Voronoi-Informed Initialization and Crossing Repair

**Status:** Approved for implementation
**Date:** 2026-05-28
**Tracking issues:** #8 (deferred Julia-native solvers), #9 (deferred Bentley-Ottmann)
**Revision:** r3 — reviewer-converged version. r1 → r2 changed dependency to `DelaunayTriangulation.jl`, corrected diagonal Imhof slot padding for AABB semantics, weakened the sparse-data non-crossing claim, re-grounded the termination proof on label-center distances, added `dropped` plumbing through the crossings pass, specified `only_move`/n<3/NaN handling, and added release-prep items. r2 → r3 softened the strict-decrease claim for the collinear-degenerate case, anchored the NaN/Inf guard at `voronoi_cells`, rewrote the integration test snippet to use real API shape, noted the `clip_polygon` Tuple form, and trimmed two aspirational items.

## Motivation

v0.1 of MakieTextRepel.jl ships a force-directed solver that places labels by iterating overlap, anchor, and pull forces from a deterministic golden-angle initialization. It has two structural shortcomings:

1. Labels can end up with **crossing leader lines**. Crossed leaders are visually confusing and are tracked as an open deficiency in ggrepel (issue #34 upstream).
2. The initial offset for each label ignores **available empty space around the anchor**. The solver has to iterate from an arbitrary starting position even when geometry already implies a good placement.

v0.2 delivers two improvements under the existing `textrepel!` API:

- **Voronoi-informed initialization** — place each label at its highest-preference Imhof slot that fits inside its anchor's Voronoi cell.
- **Crossing repair** — a deterministic post-solver pass that swaps label positions to eliminate any remaining leader-line crossings.

Together they produce **typically** non-crossing layouts on sparse data (because the force solver finds little to do from a near-equilibrium initialization) and **guaranteed** crossing-free layouts on all data (because the swap-repair backstop runs uniformly). The hard non-crossing guarantee comes from the repair pass; Voronoi initialization is a quality improvement, not a hard constraint.

## Goals

- Eliminate leader-line crossings in all rendered output (hard guarantee, provided by the repair pass).
- Use Imhof-preferred slot positions (TR > R > T > BR > L > BL > B > TL) when geometry allows.
- Preserve v0.1's determinism contract: same input → bit-exact same output.
- No new public attributes; v0.1 caller code runs unchanged.
- Lay an internal seam (`AbstractClusterSolver`) so future Julia-native solver implementations slot in without API churn.

## Non-goals (deferred or explicitly out)

- Public solver-selection attribute (`solver = :force | :ode | …`) — deferred to v0.3+, tracked in issue #8.
- AD-driven custom obstacle functions (`obstacles = [x -> …]`) — deferred to v0.3+, tracked in issue #8.
- Bentley-Ottmann sweep-line crossing detection — deferred until n ≥ ~2000 demand emerges, tracked in issue #9.
- Recovery of `max_overlaps`-dropped labels after the swap pass — **out**, user controls drop policy via input subsetting.
- Obstacle-awareness for plots on the same axis beyond peer labels — separate future feature.

## Architecture

A single solver pass that takes Voronoi cell geometry as structural input. No staged pipeline.

```
1. Compute Voronoi cells of anchors (once, via DelaunayTriangulation.jl).
2. Initialize each label at the best Imhof slot that fits its cell.
3. Run solve_repel on all labels (unchanged from v0.1).
4. Scan for residual leader crossings + position-swap repair pass.
```

Every label goes through every step. Voronoi cells inform initialization but are not used during force iteration. On sparse data the force solver typically does little work from the Voronoi-informed initial positions; on dense data the force solver carries the layout and the repair pass cleans up. The repair pass is the load-bearing guarantee — Voronoi initialization is a quality boost, not a safety property.

### Module layout

```
src/
  recipe.jl          — public API, reactive graph (light edits to call new pipeline)
  solver.jl          — solve_repel signature gains initial_offsets argument; internals unchanged
  solvers/
    abstract.jl      — NEW. AbstractClusterSolver interface (internal, not exported)
    force.jl         — NEW. ForceSolver <: AbstractClusterSolver wraps solve_repel
  voronoi.jl         — NEW. Cell computation + cell-fit predicate
  init.jl            — NEW. Imhof slot selection given cells
  crossings.jl       — NEW. connector_for + find_crossings + repair_crossings!
  geometry.jl        — existing, with helpers added
  connectors.jl      — existing, factored to share connector_for
```

**On the `AbstractClusterSolver` abstraction.** This adds a type and a wrapper for a single concrete implementation in v0.2 — a fair YAGNI critique. We are keeping it because it's the seam for issue #8's planned Julia-native solver implementations (ODE-based, AD-energy-based), and adding it now lets `solve_repel`'s call site stabilize at the new dispatch shape before a second implementation lands, avoiding a more invasive refactor later. The cost is small: an abstract type, a one-field wrapper struct, and one method.

## Detailed design

### Voronoi cell computation (`src/voronoi.jl`)

Uses `DelaunayTriangulation.jl` (v1.6+). One new dependency.

Choice rationale (r2 revision): `VoronoiCells.jl` was considered first but rejected: it pins `GeometryBasics = "0.4"` and this repo requires `GeometryBasics = "0.5"`; its API returns `Vector{Vector{Point}}` not GeometryBasics polygons; its underlying `VoronoiDelaunay.jl` reorders inputs via an internal RNG (deterministic only with an explicit seed); and the package has been stagnant since 2023. DelaunayTriangulation.jl has native clipped-Voronoi support, broad compat, active maintenance, and is the modern choice (JOSS paper, v1.6+).

```julia
voronoi_cells(anchors::Vector{Point2f}, viewport::Rect2f; rng = MersenneTwister(0))
    -> Vector{Union{Polygon, Nothing}}
```

**Implementation:**
- **Finite-anchor filter (guard site).** Pre-pass: for each anchor, if any coordinate is non-finite (NaN/Inf), record `nothing` in that slot and exclude it from the triangulation input. This is the single guard for non-finite input in the v0.2 pipeline; no upstream caller is required to pre-filter.
- For `n < 3` finite anchors: return `[nothing for _ in 1:n]`. All labels fall back to TR Imhof slot.
- For `n ≥ 3` finite anchors: sort `(anchors, recorded_index)` lexicographically by `(x, y)`, dedup coincident anchors (those collisions get `nothing` in their output slots), pass the sorted unique anchors to `DelaunayTriangulation.triangulate` with the explicit `rng`, call `voronoi(tri; clip = true, clip_polygon = viewport_clip)` where `viewport_clip` is the CCW `(points, boundary_nodes)` Tuple form required by DT.jl's API (not a `Polygon` object), then un-sort the resulting cells to the original index order.
- Returns `Polygon` for each anchor that has a real cell; `nothing` for coincident, non-finite, or filtered-out anchors.

**Helper for cell-fit testing:**

```julia
box_inside_polygon(box::Rect2f, poly::Polygon)::Bool
```

Convex point-in-polygon test on all four box corners. Voronoi cells are convex, and intersecting with a convex viewport rectangle preserves convexity, so the test is valid throughout. Implementation: sign of cross products against each polygon edge; all four corners must lie on the same (interior) side of every edge. Average Voronoi cell has small constant edge count (~5–7 in unclipped diagrams, slightly more near hull-clipped cells). Effective cost is constant per anchor at our n.

### Imhof slot initialization (`src/init.jl`)

For a label of pixel size `(w, h)` around anchor `A`, with `p = point_padding`, the eight slot offsets (label center minus anchor) are:

| Slot | Offset |
|------|--------|
| TR (NE) | `(p + w/2, p + h/2)` |
| R (E)   | `(p + w/2, 0)` |
| T (N)   | `(0, p + h/2)` |
| BR (SE) | `(p + w/2, −p − h/2)` |
| L (W)   | `(−p − w/2, 0)` |
| BL (SW) | `(−p − w/2, −p − h/2)` |
| B (S)   | `(0, −p − h/2)` |
| TL (NW) | `(−p − w/2, p + h/2)` |

All eight slots place the label so that the anchor lies outside the **axis-aligned** padded box (the same AABB semantics that `point_push` at `src/geometry.jl:39-45` uses). r1 had diagonal slots at Euclidean distance `p/√2`, which left the anchor *inside* the padded AABB of its own label — triggering `point_push` even at the supposedly-canonical position. Corrected in r2.

Preference order (Imhof 1962, refined by Liao et al. 2024): **TR > R > T > BR > L > BL > B > TL**. The order is a compile-time constant; not exposed as an attribute in v0.2.

```julia
initial_offsets(anchors, sizes, cells, params)::Vector{Point2f}
```

For each anchor:
- Iterate slots in preference order.
- The first slot for which the padded box fits entirely inside the corresponding cell is selected.
- If no slot fits (cell is `nothing`, cell too small, etc.), fall back to TR.
- After slot selection, apply `_constrain(offset, params.only_move)` so the existing `only_move = :x | :y | :both` semantics are honored from the very first iteration.

This replaces `init_offsets(n, psizes)` at `src/solver.jl:36-47`. The `_GOLDEN_ANGLE` constant (`src/solver.jl:19`) and its callers are removed.

### Solver abstraction (`src/solvers/`)

```julia
abstract type AbstractClusterSolver end

struct ForceSolver <: AbstractClusterSolver
    params::RepelParams
end

solve_cluster(s::AbstractClusterSolver,
              anchors::Vector{Point2f},
              sizes::Vector{Vec2f},
              init_offsets::Vector{Point2f},
              bounds::Rect2f)::Tuple{Vector{Point2f}, BitVector}
```

`ForceSolver` is the single concrete implementation in v0.2. It dispatches `solve_cluster` to a lightly-wrapped `solve_repel` that accepts pre-computed `init_offsets` instead of generating golden-angle initialization internally. The returned `BitVector` is the `dropped` flags from `solve_repel`'s existing `max_overlaps` logic.

The interface is **internal-only in v0.2**: not exported, not surfaced as a `textrepel!` attribute. v0.3 will expose a public `solver = …` selector once a second implementation (e.g., `ODESolver` per issue #8) exists.

### Crossing detection and repair (`src/crossings.jl`)

#### Shared connector geometry

```julia
struct Connector
    label_end::Point2f      # clip_to_box_edge(box, anchor)
    anchor_end::Point2f     # anchor trimmed by point_padding along leader
    drawn::Bool             # mirrors build_connectors suppression, INCLUDING dropped
end

connector_for(anchor, offset, size, dropped::Bool, params)::Connector
```

The `drawn` flag captures four conditions matching `src/connectors.jl:21-30` exactly:

1. `dropped == true` (label was dropped by `max_overlaps`)
2. Anchor inside padded box (`edge === nothing`)
3. Trim inverts direction (`dlen <= ppad`)
4. Visible length below `min_segment_length`

The `dropped` parameter is new in v0.2 and is what makes the `drawn` flag a faithful predicate. Both `build_connectors` (`src/connectors.jl:12-34`) and `repair_crossings!` call `connector_for` with the dropped flags vector indexed per label — single source of truth.

#### Crossing predicate

```julia
segments_cross(p1, p2, p3, p4)::Bool
```

Strict crossing via signed-area orientation tests. Returns `true` only when the segments properly intersect — endpoint-touching and collinear-overlap are **not** crossings.

#### Detection

```julia
find_crossings(connectors)::Vector{Tuple{Int,Int}}
```

Naive O(n²) pairwise iteration over `drawn == true` connectors, returning lex-ordered `(i, j)` pairs with `i < j`. At our target n ≤ 500 this is ~125k tests at ~5ms per scan — invisible against Makie's render time. Bentley-Ottmann replacement is tracked in issue #9.

#### Repair

```julia
function repair_crossings!(offsets, anchors, sizes, dropped, params; max_iter = 100)
    for iter in 1:max_iter
        connectors = [connector_for(anchors[i], offsets[i], sizes[i], dropped[i], params)
                      for i in eachindex(offsets)]
        crossings = find_crossings(connectors)
        isempty(crossings) && return iter - 1

        swapped = Set{Int}()
        for (i, j) in crossings
            (i ∈ swapped || j ∈ swapped) && continue
            swap_positions!(offsets, anchors, i, j)
            push!(swapped, i, j)
        end
    end
    max_iter
end
```

`swap_positions!` exchanges the **absolute positions** of labels `i` and `j` while preserving the (label_text → anchor) identity:

```julia
pos_i_old = anchors[i] + offsets[i]
pos_j_old = anchors[j] + offsets[j]
offsets[i] = pos_j_old - anchors[i]
offsets[j] = pos_i_old - anchors[j]
```

**Termination argument.** Consider the sum `S = Σᵢ ‖offsetᵢ‖` — total label-center-to-anchor distance. When segments `(a → A)` and `(b → B)` cross (where A, B are label centers, a, b are anchors), the triangle inequality on the fixed anchor points gives `‖a − A‖ + ‖b − B‖ ≥ ‖a − B‖ + ‖b − A‖`, with strict inequality in general position. A `swap_positions!(i, j)` exchanges A and B in the sum, so `S` is **non-increasing** per swap, and strictly decreasing whenever the four points are not collinear. The collinear case (measure-zero in continuous coordinates, possible at integer/snap-pixel coordinates) makes `S` stationary across a swap.

Termination is therefore **not** guaranteed by `S` strictly decreasing in all cases. The outer iteration cap `max_iter = 100` is the load-bearing safety mechanism that handles both (a) the collinear-degenerate case where `S` is non-strict, and (b) the batched-swap interaction below where one outer iteration's net effect on `S` may not be strictly decreasing. In practice the inner iteration converges in 1–3 passes; the cap fires only on pathological inputs.

**On non-conflicting batched swaps.** Within one outer iteration, swaps for index-disjoint pairs `(i, j)` and `(k, l)` are applied without re-scanning. Each individual swap was valid at scan time and individually does not increase `S`. Their composition can introduce a new crossing between the now-moved label `i` (or `j`) and the unchanged labels `k`, `l`; in adversarial cases the net effect on `S` across an outer iteration is not strictly decreasing. New crossings introduced this way are detected and repaired on the next outer scan; in pathological cases `max_iter` terminates the loop. The non-conflicting batch is a heuristic for per-iteration progress, not a correctness invariant.

**Interaction with existing suppression.** Connectors with `drawn == false` (dropped, anchor inside padded box, sub-min-length, inverted trim) are excluded from the scan — they can't cross anything by definition.

**Interaction with box overlap.** A swap can introduce box overlap that wasn't present before; we accept this because crossings are perceptually more harmful than mild overlap, and the existing `max_overlaps` machinery still drops labels that are completely jammed. Note: `max_overlaps` is evaluated *during* `solve_repel`, not after `repair_crossings!`. The post-repair box-overlap state is not re-evaluated against `max_overlaps`.

## Input contract

- **NaN/Inf in anchor positions:** filtered at `voronoi_cells` (see the finite-anchor pre-pass in the Voronoi section). Non-finite anchors get `nothing` cells, fall through to TR Imhof slot, and proceed through the rest of the pipeline. They will still produce non-finite offsets downstream — diagnostics are out of scope for v0.2, matching v0.1's posture, but the pipeline does not panic on them.
- **Anchors with pixel-space coordinates outside the viewport:** pass through. Voronoi computation handles them (cells outside the clip region come back empty/`nothing` → TR fallback). Final offsets pass through the existing `clamp_box_offset` machinery. Same as v0.1.
- **Coincident anchors:** detected by dedup pre-pass; both anchors get `nothing` cells → TR Imhof fallback for both → force solver disambiguates via existing label-label repulsion.
- **`n = 0`:** entire pipeline short-circuits; same posture as v0.1.
- **`n = 1` or `n = 2`:** `voronoi_cells` returns `[nothing, nothing]`; all labels fall back to TR; solver runs trivially; no crossings possible.

## Reactivity

The recipe's single `lift` node already depends on `(anchors, sizes, params, bounds_obs)` in v0.1 (`src/recipe.jl:74-89`). The new pipeline is pure over the same observables — Voronoi computation, initialization, solver call, and repair pass are all pure functions of those four inputs. **No new reactive dependencies are added.** A single `lift` continues to drive the whole pipeline.

## Backward compatibility

- No new public attributes. Every v0.1 `textrepel!` call works unchanged.
- Output offsets WILL differ from v0.1 (golden-angle → Imhof init, plus repair pass).
- Existing solver determinism tests need rebaselining (record new expected values).
- The change is strictly toward better layouts: shorter average leader lengths, no crossings.

### Release prep

- **Version bump:** `Project.toml` version from `0.1.x` to `0.2.0`.
- **CHANGELOG:** the repo does not currently have a `CHANGELOG.md`. v0.2 creates one at the repo root; the v0.2 entry is the first item.
- **Dependency:** add `DelaunayTriangulation` to `[deps]` and a compat bound `DelaunayTriangulation = "1.6"` in `[compat]`. Run `Pkg.update()` to regenerate `Manifest.toml`.
- **`[sources]` workaround** (per memory `makietextrepel-release-blockers`): unrelated to this design; tracked separately.

### CHANGELOG entry for v0.2

> Layouts are now initialized using Imhof-preferred slots within each anchor's Voronoi cell when geometry allows, and a post-solve repair pass guarantees no crossing leader lines. Existing user code runs unchanged; output positions will differ from v0.1.

## Testing strategy

```
test/
  runtests.jl          — existing, orchestrates
  test_geometry.jl     — existing, no changes
  test_solver.jl       — existing, rebaseline determinism values
  test_connectors.jl   — existing, light edits for connector_for factoring
  test_measure.jl      — existing, no changes
  test_integration.jl  — existing, rebaseline visual smoke + add invariant test
  test_init.jl         — NEW. Covers voronoi_cells AND Imhof slot selection.
  test_crossings.jl    — NEW. Covers segments_cross, find_crossings, swap_positions!, repair_crossings!.
```

### Unit tests

**`test_init.jl`** (covers `voronoi.jl` and `init.jl`):
- Canonical 3-anchor triangle produces 3 cells sharing perpendicular-bisector edges.
- Hull anchors get cells clipped to viewport (no unbounded results).
- Coincident anchors → `nothing` for both.
- `n < 3`: returns all-`nothing`.
- NaN/Inf anchor: produces `nothing` cell for that index, no exception, label falls through to TR Imhof slot.
- `box_inside_polygon` truth table: corner strictly inside / on edge / strictly outside.
- 8 slot offsets for known `(point_padding, w, h)` match expected vectors. (Verifies the AABB-semantics correction from r2.)
- When all cells are `nothing`, every label gets the TR slot.
- When only the L slot fits a contrived small cell, L is selected.
- **`only_move = :x` and `:y` propagate through initialization:** initial offsets have zero y-component (or x-component) before being passed to the solver.
- Pure function: same inputs → same outputs across repeated calls.

**`test_crossings.jl`**:
- `segments_cross` truth table: strict crossing / parallel / collinear / endpoint-touch / one endpoint on other segment.
- `find_crossings` returns lex-ordered `(i, j)` pairs with `i < j`.
- `swap_positions!` post-condition: absolute positions exchanged, anchor identities preserved.
- **Load-bearing invariant test:** hand-constructed 2-label crossing → 0 crossings out, total label-center-to-anchor distance strictly decreased.
- Termination: pathological input hits `max_iter` without hanging.
- Suppressed connectors (`drawn == false`) are excluded from the scan, including when `dropped[i] == true`.
- **`max_overlaps` interaction:** run pipeline with `max_overlaps = 2` on a crowded input; assert that dropped labels' positions are not corrupted by `repair_crossings!` (their offsets may change but they remain marked dropped and don't render).

### Integration invariant test (in `test_integration.jl`)

```julia
@testset "pipeline invariants" for case in (sparse_case, dense_case, mixed_case)
    fig = Figure()
    ax = Axis(fig[1, 1], limits = case.limits)
    plt = textrepel!(ax, case.anchors; text = case.labels, case.kwargs...)
    Makie.update_state_before_display!(fig)        # forces lift evaluation

    offsets = plt.offsets[]                         # reads the converged Observable
    dropped = plt.dropped[]
    sizes   = plt.sizes[]
    px_anchors = plt.px_anchors[]
    params  = plt.params[]

    connectors = [connector_for(px_anchors[i], offsets[i], sizes[i], dropped[i], params)
                  for i in eachindex(offsets)]

    @test all(isfinite, offsets)
    @test all(i -> dropped[i] || within_bounds(offsets[i], case.viewport), eachindex(offsets))
    @test isempty(find_crossings(connectors))
end
```

The "no crossings" assertion is the load-bearing behavioral guarantee of v0.2. `case` is a NamedTuple defined per fixture (sparse, dense, mixed) carrying `anchors`, `labels`, `limits`, `viewport`, and any per-case keyword overrides. The Observable names (`plt.offsets`, `plt.dropped`, `plt.sizes`, `plt.px_anchors`, `plt.params`) match what `recipe.jl` will register on the plot — they are part of this design.

### Out of scope for CI

- **No pixel-exact PNG regression.** Too fragile across Cairo/freetype/Makie versions, breaks for legitimate rendering changes, doesn't catch any class of bug the invariant test misses.
- Maintainer runs `examples/readme_example.jl` manually before tagging v0.2 to eyeball-confirm output.

## Determinism contract

Same `(anchors, sizes, params)` → bit-exact same offsets, holding across BLAS thread counts and Julia patch versions on the same platform.

Per-stage mechanism:

| Stage | Determinism source |
|-------|---------------------|
| Voronoi cells | Sorted unique anchor input + explicit `rng = MersenneTwister(0)` passed to `DelaunayTriangulation.triangulate` → stable cell output; un-sort by recorded index |
| Imhof init | Fixed compile-time slot ordering, integer tiebreaks, no float-equality compares |
| `solve_repel` | Existing — pure forces, no RNG, index-ordered pair iteration `for i in 1:n, j in i+1:n` |
| `find_crossings` | Lex-ordered iteration, returns sorted `Vector{Tuple{Int,Int}}` |
| `repair_crossings!` | Iterates `find_crossings`' sorted vector. The `swapped::Set{Int}` is used only for `∈` membership tests and is never iterated, so its internal hash order is irrelevant to output determinism. |

Single-threaded throughout. No BLAS-reduction-order dependencies in hot paths — the solver and crossing detection are pure Julia arithmetic on `Vec2f`.

## References

- Christensen, Marks & Shieber (1995), "An Empirical Study of Algorithms for Point-Feature Label Placement", ACM TOG. https://merl.com/publications/docs/TR94-12.pdf
- Imhof (1962/1975), cartographic label placement preference ordering.
- Liao et al. (2024), "From Top-Right to User-Right: Perceptual Prioritization of Point-Feature Label Positions". https://arxiv.org/abs/2407.11996
- Demaine et al., non-crossing matchings of points with geometric objects. https://erikdemaine.org/papers/MatchingPoints_CGTA/paper.pdf
- Van Leeuwen-Schoone, 2-opt termination for non-crossing matchings. https://arxiv.org/pdf/1202.4146
- ggrepel issue #34 (upstream anti-crossing motivation). https://github.com/slowkow/ggrepel/issues/34
- ggrepel issue #127 (Voronoi feature request upstream). https://github.com/slowkow/ggrepel/issues/127
- `DelaunayTriangulation.jl`. https://github.com/JuliaGeometry/DelaunayTriangulation.jl
