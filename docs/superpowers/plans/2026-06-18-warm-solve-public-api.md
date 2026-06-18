# warm_solve Public API Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give per-frame animation consumers (the TextMeasure "Atlas" demo) a stable, exported, stateless warm-start placement primitive without exposing the internal `AbstractClusterSolver` seam.

**Architecture:** Add one exported function `warm_solve(anchors, sizes, bounds; init_state, pin_mask, pinned_offsets, obstacles, <config kwargs>)` that builds a `RepelParams` + `ProjectionSolver` internally and forwards to the existing internal `solve_cluster`, returning its `(; offsets, dropped, iter, residual)` tuple verbatim. Separately, thread two new optional attributes (`init_state`, `obstacles`) through the `textrepel!` recipe so animated plots can warm-start in place. Document the (already-real, already-tested) determinism guarantee on the public surface.

**Tech Stack:** Julia 1.11, Makie 0.24, GeometryBasics. Tests use the in-tree `@testset` harness + CairoMakie (test-only `[extra]`).

## Global Constraints

- Keep pure layers Makie-free; `warm_solve` lives above the solver seam and may use Makie/GeometryBasics types (`Point2f`, `Vec2f`, `Rect2f`) but no `Scene`/recipe types.
- Do **not** export `solve_cluster`, `ProjectionSolver`, `RepelParams`, or the `AbstractClusterSolver` seam — issue #8 keeps the seam internal; `warm_solve` is the public face.
- `warm_solve`'s default `point_padding` is `5.0` (the user-surface marker-clearance default, matching the recipe attr and `TextRepelAlgorithm`), NOT the `RepelParams` primitive default of `0.0`.
- Return shape is the documented `(; offsets, dropped, iter, residual)` — forward `solve_cluster`'s tuple unchanged; do not reshape to `(; offsets, dropped)`.
- Package version stays `0.1.0`. Do not bump or coin a version label.
- Determinism wording must be backed by the existing passing test `"ProjectionSolver: deterministic"` (`test/test_projection_solver.jl`) — claim only what that test proves (same inputs ⇒ byte-identical `offsets`/`dropped`), and note the one carve-out: exactly-coincident anchors still fan out deterministically (this is reproducible, not nondeterministic).
- Run the suite once, tee to `test/output/test-<agent-id>.log`, then grep — never re-run without a code change (per CLAUDE.md). `test/output/` is gitignored.
- **Worktree `[sources]` trap (execution caveat):** this package's `Project.toml` `[sources]` points at the sibling `../TextMeasure.jl` by relative path. If you execute this plan inside a git worktree (e.g. under `.claude/worktrees/`), running `Pkg.test()`/`Pkg.resolve()` can **rewrite that relative `[sources]` path** — the exact failure that bit PR #11. Before any `Pkg.test()` from a worktree, symlink the sibling `../TextMeasure.jl` checkout beside the worktree, and never commit a rewritten `Project.toml` `[sources]`. Running in the main checkout sidesteps this entirely.

---

### Task 1: `warm_solve` public wrapper + export

**Files:**
- Create: `src/warm_solve.jl`
- Modify: `src/MakieTextRepel.jl` (export line `:8`; add an `include` after `solvers/projection.jl` at `:31`)
- Create test: `test/test_warm_solve.jl`
- Modify: `test/runtests.jl` (add one `include` line)

**Interfaces:**
- Consumes (internal, already defined): `RepelParams` (`src/params.jl`), `ProjectionSolver` + `solve_cluster` (`src/solvers/projection.jl`).
- Produces (new public): `warm_solve(anchors::Vector{Point2f}, sizes::Vector{Vec2f}, bounds::Rect2f; init_state::Union{Nothing,Vector{Vec2f}}=nothing, pin_mask::Union{Nothing,BitVector}=nothing, pinned_offsets::Vector{Vec2f}=Vec2f[], obstacles::Vector{Rect2f}=Rect2f[], only_move::Symbol=:both, box_padding::Real=4.0, point_padding::Real=5.0, min_segment_length::Real=2.0) -> (; offsets::Vector{Vec2f}, dropped::BitVector, iter::Int, residual::Float32)`.

- [ ] **Step 1: Write the failing test**

Create `test/test_warm_solve.jl`:

```julia
using MakieTextRepel
using MakieTextRepel: warm_solve, solve_cluster, ProjectionSolver, RepelParams
using GeometryBasics
using Test

# A small, well-separated fixture (sparse → fresh solve leaves small offsets).
const WS_BOUNDS  = Rect2f(0, 0, 400, 400)
const WS_ANCHORS = Point2f[(60, 60), (200, 80), (120, 260), (320, 200), (260, 330)]
const WS_SIZES   = Vec2f[(40, 16) for _ in 1:5]

@testset "warm_solve" begin
    @testset "fresh solve: shape + invariants" begin
        r = warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS)
        # Documented return shape: the full 4-tuple, forwarded from solve_cluster.
        @test propertynames(r) == (:offsets, :dropped, :iter, :residual)
        @test r.offsets isa Vector{Vec2f}
        @test r.dropped isa BitVector
        @test length(r.offsets) == length(WS_ANCHORS)
        @test length(r.dropped) == length(WS_ANCHORS)
        @test r.iter isa Int
        @test r.residual isa Float32
        @test all(o -> all(isfinite, o), r.offsets)
    end

    @testset "deterministic: same inputs ⇒ identical output" begin
        a = warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS)
        b = warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS)
        @test a.offsets == b.offsets
        @test a.dropped == b.dropped
    end

    @testset "equivalent to a direct solve_cluster call (seam tie)" begin
        # warm_solve must be a transparent forward over the internal primitive with
        # the same config. Build the identical RepelParams the wrapper builds.
        p = RepelParams(; only_move = :both, box_padding = 4.0,
                          point_padding = 5.0, min_segment_length = 2.0)
        direct = solve_cluster(ProjectionSolver(p), WS_ANCHORS, WS_SIZES, WS_BOUNDS)
        r = warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS)
        @test r.offsets == direct.offsets
        @test r.dropped == direct.dropped
    end

    @testset "config kwargs forward (point_padding default = 5.0)" begin
        # The wrapper default point_padding is the user-surface 5.0, not RepelParams' 0.0.
        p5 = RepelParams(; only_move = :both, box_padding = 4.0,
                           point_padding = 5.0, min_segment_length = 2.0)
        p0 = RepelParams(; only_move = :both, box_padding = 4.0,
                           point_padding = 0.0, min_segment_length = 2.0)
        d_default = solve_cluster(ProjectionSolver(p5), WS_ANCHORS, WS_SIZES, WS_BOUNDS)
        d_zero    = solve_cluster(ProjectionSolver(p0), WS_ANCHORS, WS_SIZES, WS_BOUNDS)
        @test warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS).offsets == d_default.offsets
        @test warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS;
                         point_padding = 0.0).offsets == d_zero.offsets
        # Guard against "point_padding silently ignored": the two configs must
        # actually produce different placements on this fixture.
        @test d_default.offsets != d_zero.offsets
    end

    @testset "warm-start path forwards init_state" begin
        # Warm-start from the fresh result: a second relax pass on an already-legal
        # layout reproduces solve_cluster's warm path exactly.
        fresh = warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS)
        p = RepelParams(; only_move = :both, box_padding = 4.0,
                          point_padding = 5.0, min_segment_length = 2.0)
        direct = solve_cluster(ProjectionSolver(p), WS_ANCHORS, WS_SIZES, WS_BOUNDS;
                               init_state = fresh.offsets)
        warm = warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS; init_state = fresh.offsets)
        @test warm.offsets == direct.offsets
        @test warm.dropped == direct.dropped
    end

    @testset "pin + obstacles forward" begin
        pin = falses(length(WS_ANCHORS)); pin[1] = true
        pinned = Vec2f[Vec2f(0, 0) for _ in 1:length(WS_ANCHORS)]
        pinned[1] = Vec2f(30, 30)
        obs = Rect2f[Rect2f(100, 100, 60, 40)]
        p = RepelParams(; only_move = :both, box_padding = 4.0,
                          point_padding = 5.0, min_segment_length = 2.0)
        direct = solve_cluster(ProjectionSolver(p), WS_ANCHORS, WS_SIZES, WS_BOUNDS;
                               pin_mask = pin, pinned_offsets = pinned, obstacles = obs)
        warm = warm_solve(WS_ANCHORS, WS_SIZES, WS_BOUNDS;
                          pin_mask = pin, pinned_offsets = pinned, obstacles = obs)
        @test warm.offsets == direct.offsets
        @test warm.offsets[1] == Vec2f(30, 30)   # pinned label held at its fixed offset
    end
end
```

- [ ] **Step 2: Register the test file**

In `test/runtests.jl`, add the include after the `test_projection_solver.jl` line (`:15`):

```julia
    include("test_projection_solver.jl")
    include("test_warm_solve.jl")
    include("test_annotation_algorithm.jl")
```

- [ ] **Step 3: Run the test to verify it fails**

```bash
LOG="test/output/test-warmsolve-t1.log"
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -nE "warm_solve|UndefVarError|Test Summary|Fail|Error" "$LOG"
```
Expected: FAIL — `warm_solve` is not defined / not exported (`UndefVarError: warm_solve`).

- [ ] **Step 4: Write the implementation**

Create `src/warm_solve.jl`:

```julia
# warm_solve.jl — the public stateless warm-start placement primitive.
#
# A thin, render-free face over the internal ProjectionSolver/solve_cluster seam
# (kept internal per issue #8). Built for consumers that re-solve placement on
# every frame (e.g. animated zoom-dives) and want to warm-start from the previous
# frame's offsets — relax, not re-seed — keyed by their own stable ids.

"""
    warm_solve(anchors, sizes, bounds;
               init_state = nothing, pin_mask = nothing,
               pinned_offsets = Vec2f[], obstacles = Rect2f[],
               only_move = :both, box_padding = 4.0,
               point_padding = 5.0, min_segment_length = 2.0)
        -> (; offsets::Vector{Vec2f}, dropped::BitVector, iter::Int, residual::Float32)

Place `length(anchors)` text labels around their `anchors` using the default
`ProjectionSolver` (Voronoi seed → side-select → crossing repair → constraint-
projection legalize → geometric over-capacity drop), returning per-label pixel
offsets and a drop mask. This is the low-level placement primitive that the
`textrepel!` recipe and `TextRepelAlgorithm` use internally, exposed as a stable
public function decoupled from the render/annotation machinery.

All distances are in pixels and all geometry is in one consistent pixel space.

# Arguments
- `anchors::Vector{Point2f}` — the data points, projected to pixel space.
- `sizes::Vector{Vec2f}` — each label's *unpadded* `(width, height)` in px (e.g.
  from `MakieTextRepel.measure_labels`). Same length as `anchors`.
- `bounds::Rect2f` — the clamp region (axis viewport) in the same pixel space.

# Keyword arguments
- `init_state` — `nothing` ⇒ fresh placement (the solver seeds and repairs
  itself). A `Vector{Vec2f}` of per-label offsets ⇒ **warm-start**: relax that
  layout in place (legalize only), which is what keeps per-frame animation smooth.
- `pin_mask` / `pinned_offsets` — `pin_mask[i] == true` holds label `i` fixed at
  `pinned_offsets[i]` while its box still repels the others. Both must have length
  `length(anchors)` when `pin_mask` is given.
- `obstacles::Vector{Rect2f}` — extra keep-out boxes the labels must avoid.
- `only_move` (`:both` | `:x` | `:y`), `box_padding`, `point_padding`,
  `min_segment_length` — the `ProjectionSolver`-relevant `RepelParams` knobs.
  `point_padding` is the marker-clearance gap (default `5.0`, matching the
  `textrepel!` and `TextRepelAlgorithm` surfaces).

# Determinism
Deterministic: identical inputs always produce byte-identical `offsets` and
`dropped` (no RNG — the one randomized dependency, DelaunayTriangulation, is
seeded with `MersenneTwister(0)` and its inputs are lexicographically sorted).
Consumers may golden-test placement directly. (Exactly-coincident anchors fan out
along a fixed golden-angle spiral — still fully reproducible.)

See also [`textrepel!`](@ref), [`TextRepelAlgorithm`](@ref).
"""
function warm_solve(anchors::Vector{Point2f}, sizes::Vector{Vec2f}, bounds::Rect2f;
                    init_state::Union{Nothing,Vector{Vec2f}} = nothing,
                    pin_mask::Union{Nothing,BitVector}       = nothing,
                    pinned_offsets::Vector{Vec2f}            = Vec2f[],
                    obstacles::Vector{Rect2f}                = Rect2f[],
                    only_move::Symbol         = :both,
                    box_padding::Real         = 4.0,
                    point_padding::Real       = 5.0,
                    min_segment_length::Real  = 2.0)
    params = RepelParams(; only_move = only_move,
                           box_padding = Float64(box_padding),
                           point_padding = Float64(point_padding),
                           min_segment_length = Float64(min_segment_length))
    return solve_cluster(ProjectionSolver(params), anchors, sizes, bounds;
                         init_state = init_state, pin_mask = pin_mask,
                         pinned_offsets = pinned_offsets, obstacles = obstacles)
end
```

- [ ] **Step 5: Wire export + include**

In `src/MakieTextRepel.jl`, change the export line (`:8`):

```julia
export textrepel, textrepel!, warm_solve
```

And add the include immediately after the `solvers/projection.jl` line (`:31`), so the seam types it uses are already defined:

```julia
include("solvers/projection.jl")  # ProjectionSolver — the DEFAULT, composes the stages above

# Public stateless warm-start primitive (face over the internal seam)
include("warm_solve.jl")
```

- [ ] **Step 6: Run the test to verify it passes**

```bash
LOG="test/output/test-warmsolve-t1.log"
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -nE "warm_solve|Test Summary|Fail|Error" "$LOG"
```
Expected: the `warm_solve` testset passes; overall `Test Summary` shows no new failures.

- [ ] **Step 7: Commit**

```bash
git add src/warm_solve.jl src/MakieTextRepel.jl test/test_warm_solve.jl test/runtests.jl
git commit -m "feat: add public warm_solve warm-start placement primitive (#24)"
```

---

### Task 2: Thread `init_state` + `obstacles` through the `textrepel!` recipe

**Files:**
- Modify: `src/recipe.jl` (attribute block `@recipe` `:3-56`; the solve `lift` `:76-111`)
- Modify test: `test/test_integration.jl` (append two `@testset`s)

**Interfaces:**
- Consumes: `warm_solve` is NOT used here; the recipe keeps calling `solve_cluster(ProjectionSolver(params), …)` directly (it already constructs `params`). The change is to pass `init_state`/`obstacles` into that existing call.
- Produces: two new recipe attributes `init_state` (default `nothing`) and `obstacles` (default `Rect2f[]`), both reactive inputs to the solve lift.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_integration.jl` (the `using …: solve_cluster, ProjectionSolver` symbols are reachable via `MakieTextRepel.` qualification, as the existing `#12` test does):

```julia
@testset "recipe threads obstacles into the solve (#24)" begin
    fig = Figure(); ax = Axis(fig[1, 1])
    pts = [Point2f(1, 1), Point2f(2, 2), Point2f(1.5, 2.5), Point2f(2.2, 1.1)]
    obs = Rect2f[Rect2f(40, 40, 120, 80)]   # pixel-space keep-out box
    plt = textrepel!(ax, pts; text = ["alpha", "beta", "gamma", "delta"],
                     obstacles = obs)
    Makie.update_state_before_display!(fig)

    anchors = plt.attributes[:computed_anchors][]
    sizes   = plt.attributes[:computed_sizes][]
    params  = plt.attributes[:computed_params][]
    # The recipe's solve must equal a direct solve_cluster with the SAME obstacles…
    direct = MakieTextRepel.solve_cluster(MakieTextRepel.ProjectionSolver(params),
                                          anchors, sizes, params.bounds; obstacles = obs)
    @test plt.attributes[:computed_offsets][] == direct.offsets
    # …and differ from the no-obstacle solve (proving the attr actually took effect).
    plain = MakieTextRepel.solve_cluster(MakieTextRepel.ProjectionSolver(params),
                                         anchors, sizes, params.bounds)
    @test plt.attributes[:computed_offsets][] != plain.offsets
end

@testset "recipe threads init_state (warm-start) into the solve (#24)" begin
    fig = Figure(); ax = Axis(fig[1, 1])
    pts = [Point2f(1, 1), Point2f(2, 2), Point2f(1.5, 2.5), Point2f(2.2, 1.1)]
    n = length(pts)
    warm = Vec2f[Vec2f(0, 0) for _ in 1:n]   # warm-start from "all at anchor"
    plt = textrepel!(ax, pts; text = ["alpha", "beta", "gamma", "delta"],
                     init_state = warm)
    Makie.update_state_before_display!(fig)

    anchors = plt.attributes[:computed_anchors][]
    sizes   = plt.attributes[:computed_sizes][]
    params  = plt.attributes[:computed_params][]
    direct = MakieTextRepel.solve_cluster(MakieTextRepel.ProjectionSolver(params),
                                          anchors, sizes, params.bounds; init_state = warm)
    @test plt.attributes[:computed_offsets][] == direct.offsets
    # Warm (relax-only) ≠ fresh (seed + side-select) on this crowded fixture.
    fresh = MakieTextRepel.solve_cluster(MakieTextRepel.ProjectionSolver(params),
                                         anchors, sizes, params.bounds)
    @test plt.attributes[:computed_offsets][] != fresh.offsets
end
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
LOG="test/output/test-warmsolve-t2.log"
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -nE "threads obstacles|threads init_state|MethodError|Fail|Error" "$LOG"
```
Expected: FAIL — `textrepel!` rejects the unknown `obstacles`/`init_state` attributes (or the attrs are ignored, so offsets equal the plain/fresh solve and the `!=` assertions fail).

- [ ] **Step 3: Add the two attributes**

In `src/recipe.jl`, inside the `@recipe TextRepel` block, add after the `markersize` attribute (`:25`), before the `max_overlaps` attribute:

```julia
    "Warm-start offsets (`Vector{Vec2f}`, pixel space), one per label, to relax from instead of seeding a fresh placement; `nothing` = fresh placement each solve. For animated plots that re-solve per frame, feed the previous frame's `computed_offsets` here. Length must equal the number of labels."
    init_state = nothing
    "Extra keep-out boxes (`Vector{Rect2f}`, pixel space) the solver must place labels clear of, in addition to the data markers."
    obstacles = Rect2f[]
```

- [ ] **Step 4: Thread them into the solve lift**

In `src/recipe.jl`, extend the solve `lift` (`:76-78`). Add `p.init_state, p.obstacles` to the tracked inputs and `is, obs` to the do-block argument list:

```julia
    solved = lift(p.px_anchors, p.text, p.fontsize, p.font,
                  p.force, p.force_point, p.force_pull, p.max_iter, p.only_move,
                  p.box_padding, p.point_padding, p.max_overlaps, bounds_obs, p.min_segment_length,
                  p.markersize, p.init_state, p.obstacles) do px, labels, fs, font,
                                                              fr, frp, fpl, mi, om,
                                                              bp, pp, mo, bnds, ml, ms, is, obs
```

Also update the now-stale comment block directly above the call (`:99-108`): it currently says the bounds arg is "fed straight through to the pipeline" via a bare single-arg `solve_cluster`. After this edit the call also forwards `init_state`/`obstacles`, so revise that comment to mention the two animation attrs are threaded in (default `nothing`/`Rect2f[]` ⇒ the unchanged fresh path). Do not delete the existing note explaining why `bounds` is set in both `params` and the positional arg — that rationale still holds.

Then replace the `solve_cluster` call (`:109`) with the warm-start-aware form:

```julia
        # Coerce the two animation attrs to the seam's expected types. `init_state`
        # is per-label pixel offsets (nothing = fresh); `obstacles` are pixel-space
        # keep-out boxes (empty = none).
        is_v  = is  === nothing ? nothing : Vector{Vec2f}(is)
        obs_v = obs === nothing ? Rect2f[] : Vector{Rect2f}(obs)
        offsets, dropped, _, _ = solve_cluster(ProjectionSolver(params), anchors, sizes, bnds;
                                               init_state = is_v, obstacles = obs_v)
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
LOG="test/output/test-warmsolve-t2.log"
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -nE "threads obstacles|threads init_state|Test Summary|Fail|Error" "$LOG"
```
Expected: both new testsets pass; `Test Summary` shows no new failures (the existing `#12` "recipe solve equals a direct solve_cluster call" test still passes because its default `init_state=nothing`/`obstacles=Rect2f[]` reproduce the fresh path).

- [ ] **Step 6: Commit**

```bash
git add src/recipe.jl test/test_integration.jl
git commit -m "feat: thread init_state + obstacles through textrepel! for animation (#24)"
```

---

### Task 3: Document the determinism guarantee + new public surface

**Files:**
- Modify: `src/solvers/abstract.jl` (docstring `:3-16`)
- Modify: `docs/algorithm.md` (determinism prose)
- Modify: `CLAUDE.md` (public-surface + project-state notes)

**Interfaces:**
- Consumes: nothing (documentation only; no code change).
- Produces: nothing executable.

- [ ] **Step 1: Add the determinism guarantee to the seam docstring**

In `src/solvers/abstract.jl`, replace the final sentence of the docstring (`:13-15`, "Callers must NOT … issue #8).") with:

```
`init_state === nothing` ⇒ fresh placement (the solver does its own init + crossing
repair); a given `init_state` ⇒ relax (warm-start, solve only). Callers must NOT
perform init/placement/repair outside `solve_cluster`. Under the default
`ProjectionSolver` the result is **deterministic** — identical inputs yield
byte-identical `offsets`/`dropped` (no RNG; DelaunayTriangulation is seeded with
`MersenneTwister(0)` over lexicographically sorted points). Internal for now; the
public stateless face is `warm_solve` (the seam itself is exposed when a second
strategy lands — see GitHub issue #8).
```

- [ ] **Step 2: Document determinism in docs/algorithm.md**

In `docs/algorithm.md`, find the existing determinism statement (the "Deterministic output" / "the same data produces the same figure" line) and extend it so consumers know they can golden-test placement. Add this sentence immediately after that statement:

```
The default `ProjectionSolver` carries no RNG: its one randomized dependency
(DelaunayTriangulation, for the Voronoi-informed seed) is seeded with
`MersenneTwister(0)` over lexicographically sorted points, and every other stage
is a pure deterministic algorithm. Identical inputs therefore produce
byte-identical offsets and drop masks, so downstream consumers of the public
`warm_solve` primitive may golden-test placement directly. (Exactly-coincident
anchors fan out along a fixed golden-angle spiral — also fully reproducible.)
```

- [ ] **Step 3: Note the new public surface in CLAUDE.md**

In `CLAUDE.md`, under "What this is", add `warm_solve` to the surfaces sentence. Change:

```
It ships two surfaces over the same deterministic solver: `textrepel!` (a standalone Makie recipe with the full feature set — solver, dropping, background boxes, connector trimming) and `TextRepelAlgorithm` (a plug-in for `Makie.annotation!` ...).
```

to append a third surface:

```
It ships three surfaces over the same deterministic solver: `textrepel!` (a standalone Makie recipe with the full feature set — solver, dropping, background boxes, connector trimming; also accepts `init_state`/`obstacles` for animated re-solves), `TextRepelAlgorithm` (a plug-in for `Makie.annotation!` ...), and `warm_solve` (the exported stateless warm-start primitive — `(anchors, sizes, bounds; init_state, pin_mask, pinned_offsets, obstacles, …) -> (; offsets, dropped, iter, residual)` — a render-free face over the internal `solve_cluster`/`ProjectionSolver` seam for per-frame animation consumers; see `src/warm_solve.jl`).
```

Then in the "Project state & history" section, append to the bullet that tracks deferred issues a note that issue #24 is addressed:

```
Issue #24 (public warm-start primitive for per-frame animation consumers) is addressed by the exported `warm_solve` (`src/warm_solve.jl`) plus the `textrepel!` `init_state`/`obstacles` attributes; the `AbstractClusterSolver` seam itself stays internal (issue #8 unchanged).
```

- [ ] **Step 4: Verify no code regressions (docs-only, so just confirm the suite is still green)**

```bash
LOG="test/output/test-warmsolve-t3.log"
julia --project=. -e 'using Pkg; Pkg.test()' 2>&1 | tee "$LOG"
grep -E "Test Summary|Fail|Error" "$LOG"
```
Expected: `Test Summary` shows all tests passing, zero fails/errors.

- [ ] **Step 5: Commit**

```bash
git add src/solvers/abstract.jl docs/algorithm.md CLAUDE.md
git commit -m "docs: document warm_solve, recipe animation attrs, determinism guarantee (#24)"
```

---

## Self-Review

**Spec coverage (issue #24):**
- Ask 1 — "thin public stateless `warm_solve` returning the documented tuple, decoupled from render/annotation": **Task 1** (returns the full `(; offsets, dropped, iter, residual)`, not just `(offsets, dropped)`, per the issue's "document the `(; offsets, dropped, iter, residual)` return"). Seam stays internal (honors #8).
- Ask 2 (secondary) — "document determinism guarantee": **Task 3** (seam docstring + algorithm.md), with the determinism backed by Task 1's `warm_solve` determinism test and the existing `ProjectionSolver: deterministic` test.
- Ask 3 (secondary) — "thread `init_state`/`obstacles` onto `textrepel!`": **Task 2**.

**Placeholder scan:** every code/test step shows complete code; commands have expected output. No TBD/TODO/"handle edge cases".

**Type consistency:** `warm_solve` signature is identical in the Interfaces block, the docstring, and the implementation (Task 1 Step 4). `init_state::Union{Nothing,Vector{Vec2f}}`, `obstacles::Vector{Rect2f}` match `solve_cluster`'s declared types (`src/solvers/projection.jl:62-65`). Recipe coercion (`Vector{Vec2f}(is)`, `Vector{Rect2f}(obs)`) produces exactly those types. Return-tuple field order `(:offsets, :dropped, :iter, :residual)` matches `solve_cluster`'s and is asserted in Task 1 Step 1.

**Carve-out honored:** the determinism wording in both Task 1's docstring and Task 3 explicitly notes the exactly-coincident-anchor golden-angle fan-out (consistent with the existing `#12` "fans out exactly-coincident anchors" test), so the guarantee isn't overstated.
