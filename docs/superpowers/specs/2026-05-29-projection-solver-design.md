# v0.3 Design — `ProjectionSolver` (side-select → legalize)

**Status:** Drafted for review
**Date:** 2026-05-29
**Research basis:** `docs/research/2026-05-29-label-solver-first-principles.md` (§7a–§7e empirical pressure-tests)
**Tracking issues:** #8 (Julia-native solver seam — this realizes it with a second `AbstractClusterSolver`)

## Motivation

The v0.2 default solver (`ForceSolver`) places labels by force-directed iteration from a Voronoi-informed Imhof initialization, then repairs crossings. A first-principles research arc (the research note above) established two structural limits of the force loop, both confirmed empirically against the live `solve_cluster`:

1. **It leaves residual box overlaps even on easy scenes.** Force balance settles where forces cancel, not where overlap is zero — §7b/§7c measured 8–31 residual overlapping pairs on scenes that are geometrically separable. Force balance is not an overlap guarantee.
2. **It produces long leader lines.** Because the continuous loop has no notion of "which side of the anchor is best", it drifts labels far from their anchors to relieve force, even when a short-leader slot was available. §7e measured leaders ~2× longer than necessary.

The research converged on a single pipeline — not a menu of architectures — that fixes both:

- **Discrete side-selection** (a deterministic greedy search over Imhof slots) chooses the *right side* per label, owning leader quality.
- **Constraint-projection legalization** (Dykstra cyclic projection) then nudges boxes the minimum distance needed to reach **provably zero overlap**, owning the overlap guarantee.

§7e's three-pipeline comparison on the live solver showed `side-select → legalize` reaching zero overlap on every feasible scene while **halving leader length** versus legalizing the force output (knot: 49.5→26.8; sparse: 37.1→12.9; collinear: 76.1→15.2 px mean leader), with equal-or-better crossing counts. The continuous force loop is *subsumed*: the discrete front-end picks the side, the legalizer guarantees separation, and there is no force balance left to do.

This design delivers that pipeline as a new `ProjectionSolver <: AbstractClusterSolver` and makes it the **default** for both surfaces (`textrepel!` and `TextRepelAlgorithm`). `ForceSolver` stays in-tree as a fallback and comparison baseline.

## Goals

- **Zero-overlap guarantee** for all in-bounds, feasible scenes (load-bearing, provided by the legalizer).
- **Shorter leader lines** than `ForceSolver` on the §7e fixture scenes (quality target, provided by side-selection).
- **Graceful over-capacity dropping**: when a scene cannot fit overlap-free within bounds, drop the most-overlapped labels deterministically and re-legalize until feasible — never silently leave overlaps.
- **Fully rng-free determinism**: same `(anchors, sizes, params)` → bit-exact same output, holding across BLAS thread counts and Julia patch versions on the same platform. No RNG anywhere in the new pipeline.
- **No new public attributes.** Every v0.2 caller runs unchanged. The pipeline swap is internal.
- **Keep the new computational layers Makie-free** (`cost.jl`, `legalize.jl`, `side_select.jl` depend only on GeometryBasics), preserving the project's pure-layer architecture and unit-testability.
- Reuse the existing validated pieces — `voronoi_cells`, `initial_offsets`, `slot_offset`, `repair_crossings!` — rather than reimplementing them.

## Non-goals (deferred or explicitly out)

- **Public `solver = :force | :projection` selector** — deferred. The swap is internal in this increment; `AbstractClusterSolver` export remains tracked in issue #8. Promotion happens once the selector API is designed.
- **Public label-priority/importance attribute** — out. Over-capacity dropping uses a deterministic geometric heuristic (most-overlapped). A user-facing priority input is a separate future feature; users still control drop policy by subsetting their input.
- **Scan-line / corrected-2007 VPSC** (constraint-DAG + block merging) — out. The Dykstra cyclic-projection legalizer reaches zero overlap on every feasible scene in the spikes at our scene sizes; the constraint-DAG machinery is unjustified complexity for now. Tracked as a future optimization if n grows past where Dykstra's per-round cost matters.
- **Bentley-Ottmann crossing detection** — out (unchanged from v0.2; still issue #9). Crossing handling reuses the existing O(n²) `repair_crossings!`.
- **Simulated annealing / any stochastic search** — out by design. The research confirmed SA wins on quality in the literature but requires RNG; the determinism constraint is satisfied without it because both side-selection and projection are deterministic.

## Architecture

A staged pipeline behind one `solve_cluster` method. Every label flows through every stage on a fresh call:

```
seed      ← initial_offsets(anchors, sizes, voronoi_cells(...))   [REUSE v0.2]
              Voronoi-informed Imhof slot per label
side-sel  ← greedy discrete slot refinement                       [NEW, pure]
              minimize (overlap-count·W + ‖offset‖) over the 8 Imhof slots
repair    ← repair_crossings! on the discrete offsets             [REUSE v0.2]
              deterministic 2-opt position swaps (the user-chosen
              "discrete swap before legalize" crossing strategy)
legalize  ← Dykstra constraint projection                         [NEW, pure]
              minimum-displacement nudge → zero overlap; drop loop
              on infeasibility
Q         ← label_cost(...)                                       [NEW, pure]
              read-only (overlaps, mean_leader, crossings) → diagnostics
```

**Why this order.** Side-selection is discrete and cheap; it gets each label onto its best side first. `repair_crossings!` swaps *absolute positions* between labels, which can only reduce total leader length (its termination argument) and resolves topology before the continuous stage. The legalizer then minimizes displacement *from its input*, so the good discrete arrangement survives nearly untouched — this is exactly why feeding it the side-selected layout (rather than the force output) yields the §7e leader-length win. Q is pure instrumentation computed last; it never feeds back.

**Warm-start / relax path.** When `solve_cluster` is called with `init_state !== nothing` (the `reset = false` path from `TextRepelAlgorithm`, or any caller relaxing an existing layout), the seed + side-select + repair stages are **skipped** and only the legalizer runs on the supplied offsets. This preserves the `AbstractClusterSolver` contract ("given `init_state` ⇒ relax, solve only") and means warm-starts get the overlap guarantee without re-deciding sides the caller already fixed.

### Module layout

```
src/
  MakieTextRepel.jl       — include the three new pure files + the new solver; no export changes
  cost.jl                 — NEW (pure). Q functional: label_cost(...)
  legalize.jl             — NEW (pure). Dykstra constraint-projection legalizer
  side_select.jl          — NEW (pure). Greedy discrete Imhof-slot refinement
  solvers/
    abstract.jl           — existing; unchanged
    force.jl              — existing; unchanged (ForceSolver stays as fallback/baseline)
    projection.jl         — NEW. ProjectionSolver <: AbstractClusterSolver
  recipe.jl               — MODIFY one line (src/recipe.jl:97): ForceSolver → ProjectionSolver
  annotation_algorithm.jl — MODIFY one line (src/annotation_algorithm.jl:178): ForceSolver →
                            ProjectionSolver; extend solve_stats to surface Q
  init.jl, voronoi.jl,
  crossings.jl, solver.jl,
  geometry.jl, connectors.jl,
  measure.jl              — existing; REUSED unchanged
```

The three new pure files mirror the existing pure layers (`geometry.jl`, `solver.jl`, `init.jl`): GeometryBasics-only, no Makie types, independently unit-testable. `projection.jl` is the only new file that touches the solver-internal types, and it is thin — it composes the pure layers and stores diagnostics.

## Detailed design

Shared conventions used below (matching the spikes and `RepelParams` defaults):

- **Padded size** of label `i`: `psize[i] = size[i] + Vec2f(2·box_padding, 2·box_padding)`. All overlap and separation math uses padded half-extents `hw[i] = psize[i].x/2`, `hh[i] = psize[i].y/2`.
- **Center** of label `i`: `c[i] = anchor[i] + offset[i]`.
- **Overlap penetration** of pair `(i,j)`: `ox = (hw[i]+hw[j]) − |c[i].x − c[j].x|`, `oy = (hh[i]+hh[j]) − |c[i].y − c[j].y|`. The pair overlaps iff `ox > ε` **and** `oy > ε`.
- **`ε` (overlap epsilon):** `0.01` px for the legalizer's internal convergence and constraint generation; `0.5` px for the Q scorer's reported overlap count (a sub-pixel touch is numerically present but not visually an overlap — this is the threshold split §7e validated to avoid the false-positive confound).

### Q functional (`src/cost.jl`)

```julia
label_cost(anchors::Vector{Point2f}, offsets::Vector{Vec2f}, sizes::Vector{Vec2f},
           bounds::Rect2f; dropped::Union{Nothing,BitVector} = nothing,
           box_padding::Real, min_segment_length::Real)
    -> (; overlaps::Int, mean_leader::Float32, crossings::Int)
```

Pure, read-only, no mutation, no feedback into placement. Computes three components separately (reported as a NamedTuple, never collapsed into one opaque scalar — research idea #3):

- **`overlaps`** — count of label pairs with penetration `> 0.5` px on both axes, skipping any label flagged in `dropped`. Padded half-extents as above.
- **`mean_leader`** — mean `‖offset[i]‖` over non-dropped labels (`0f0` if none).
- **`crossings`** — number of crossing leader pairs, computed by building `Connector`s via the existing `connector_for` (`src/crossings.jl`) and calling the existing `find_crossings`. Reuses v0.2 geometry verbatim; no new crossing math.

This is the instrument layer (research Architecture C). It exists so callers and tests can *measure* placement quality, and so over-capacity decisions and regression tests have a single source of truth. It does not influence the legalizer or side-selection (those use their own inline overlap checks for speed).

### Constraint-projection legalizer (`src/legalize.jl`)

The validated keystone (§7c). Given an arbitrary set of offsets, move each box the minimum distance needed so that no two padded boxes overlap, while holding fixed boxes (pinned labels, obstacles) in place.

```julia
legalize(anchors::Vector{Point2f}, offsets::Vector{Vec2f}, psizes::Vector{Vec2f},
         bounds::Rect2f;
         fixed::Union{Nothing,BitVector} = nothing,
         only_move::Symbol = :both,
         rounds::Int = 400)
    -> (; offsets::Vector{Vec2f}, residual::Float32, rounds_used::Int)
```

**Per round:**
1. Find all currently-overlapping pairs (penetration `> ε = 0.01` on both axes). If none, stop — record `rounds_used` and `residual = 0`.
2. Assign each overlapping pair to the **cheaper axis**: if `ox ≤ oy` separate along x, else along y (resolving the smaller penetration is the lower-displacement move). This produces two independent 1-D constraint sets `xcons`, `ycons`, each a list of `(lo, hi, gap)` with `gap = hw[i]+hw[j]` (or `hh`) and `lo`/`hi` the lower/upper-positioned index on that axis.
3. **Dykstra cyclic projection** (below) on `xcons`, then on `ycons`, updating centers in place.
4. Clamp every movable center back inside bounds (`hw[i] ≤ x[i] ≤ W−hw[i]`, likewise y).

After `rounds`, return offsets, the final maximum penetration as `residual` (Float32; `0` on success), and `rounds_used`. **`residual > 0` after the cap is the over-capacity signal** consumed by the drop loop in `projection.jl`.

**Dykstra cyclic projection** onto the half-spaces `pos[hi] − pos[lo] ≥ gap`, converging to the minimum-sum-of-squares displacement satisfying all constraints (the same QP VPSC solves) — and crucially *satisfying* the constraints rather than settling at a force balance:

```julia
function dykstra!(pos::Vector{Float64}, cons, movable::Vector{Bool}; iters = 5000, tol = 1e-3)
    m = length(cons); m == 0 && return pos
    Ilo = zeros(m); Ihi = zeros(m)          # per-constraint correction terms (Dykstra memory)
    for _ in 1:iters
        changed = 0.0
        for k in 1:m
            lo, hi, gap = cons[k]
            ylo = pos[lo] + Ilo[k]; yhi = pos[hi] + Ihi[k]
            viol = gap - (yhi - ylo)
            if viol > 0
                # split the correction by which endpoints can move
                ml, mh = movable[lo], movable[hi]
                if ml && mh
                    xlo = ylo - viol/2; xhi = yhi + viol/2
                elseif ml
                    xlo = ylo - viol;   xhi = yhi          # only lo moves
                elseif mh
                    xlo = ylo;          xhi = yhi + viol   # only hi moves
                else
                    xlo = ylo;          xhi = yhi          # both fixed: cannot satisfy (infeasible pair)
                end
            else
                xlo = ylo; xhi = yhi
            end
            Ilo[k] = ylo - xlo; Ihi[k] = yhi - xhi
            changed = max(changed, abs(pos[lo]-xlo), abs(pos[hi]-xhi))
            pos[lo] = xlo; pos[hi] = xhi
        end
        changed < tol && break
    end
    pos
end
```

**Fixed nodes.** `fixed[i] == true` (pinned labels and obstacle pseudo-nodes — see `projection.jl`) means box `i` does not move: it contributes its half-extents to constraints but receives zero displacement (the `movable` split above). A constraint between two fixed boxes that overlap is unsatisfiable and is left as residual — surfaced as over-capacity. The spike `dykstra!` (which split every violation in half) is the `movable = all-true` special case of this; the prototype's zero-overlap result is preserved for the all-movable scenes it tested.

**`only_move`.** When `only_move == :x`, no `ycons` are generated (and vice-versa for `:y`); separation is attempted only on the permitted axis. Scenes that cannot be separated on the single permitted axis terminate with `residual > 0` → drop. `:both` is the default and the common case.

**Determinism.** Index-ordered pair iteration (`for i in 1:n, j in i+1:n`) builds `cons` in a fixed order; Dykstra sweeps `cons` in that fixed order; no RNG, no float-equality branching beyond the `> ε` penetration test. Float64 internally for projection stability, converted back to `Vec2f` offsets on return.

### Discrete side-selection (`src/side_select.jl`)

The leader-quality lever (§7e). Picks, per label, which of the eight Imhof slots to occupy, minimizing overlap first and leader length second — a deterministic greedy local search seeded from the Voronoi-informed init.

```julia
side_select(anchors::Vector{Point2f}, sizes::Vector{Vec2f}, psizes::Vector{Vec2f},
            bounds::Rect2f, seed::Vector{Vec2f}, params::RepelParams;
            pin_mask::Union{Nothing,BitVector} = nothing,
            pinned_offsets::Vector{Vec2f} = Vec2f[],
            obstacles::Vector{Rect2f} = Rect2f[],
            overlap_weight::Float64 = 1000.0,
            passes::Int = 6)
    -> Vector{Vec2f}
```

**Candidate slots.** For each label `i`, the candidate offsets are the eight `slot_offset(s, sizes[i], params.point_padding)` for `s in IMHOF_ORDER`, each filtered to those whose padded box lies in bounds (if none fit, keep all eight so the legalizer can rescue it). `only_move` constrains candidates: with `:x`, only slots whose offset has zero y-component (R, L) are eligible (plus the constrained TR fallback); symmetric for `:y`.

**Seeding.** Initial selection per label = the slot whose offset is nearest the supplied `seed[i]` (the Voronoi-informed `initial_offsets` result), so the v0.2 Voronoi work is the starting point rather than discarded. Pinned labels (`pin_mask[i]`) are fixed at `pinned_offsets[i]` and never refined.

**Greedy refinement.** Up to `passes` sweeps in index order; in each sweep, for each non-pinned label, choose the candidate slot minimizing

```
cost(slot) = overlap_count(slot vs. current selection of all others + obstacles) · overlap_weight
             + ‖slot offset‖
```

where `overlap_count` uses the inline padded-box test (`overlap_push(b1,b2) != 0`, matching the spike). The high `overlap_weight` makes overlap-avoidance lexicographically dominate leader length, with leader length as the tiebreak — exactly the cost that produced the §7e win. Sweeps stop early when a full pass changes no selection (fixpoint). Pinned labels and obstacles participate as fixed boxes in every `overlap_count` but are never moved.

**Determinism.** Fixed candidate order (`IMHOF_ORDER`), index-ordered sweeps, strict `<` improvement test (first slot wins ties by IMHOF preference), no RNG.

### `ProjectionSolver` (`src/solvers/projection.jl`)

```julia
struct ProjectionSolver <: AbstractClusterSolver
    params::RepelParams
    stats::Base.RefValue{NamedTuple}    # last-solve Q diagnostics; see solve_stats
end

ProjectionSolver(params::RepelParams) =
    ProjectionSolver(params, Ref((; overlaps = 0, mean_leader = 0f0, crossings = 0,
                                    iter = 0, residual = 0f0, dropped = 0)))
```

`solve_cluster(s::ProjectionSolver, anchors, sizes, bounds; init_state, pin_mask, pinned_offsets, obstacles)` returns the contract NamedTuple `(; offsets, dropped, iter, residual)` and composes the stages:

```
p       = RepelParams(s.params; bounds = bounds)           # bounds override, as ForceSolver does
psizes  = [sizes[i] .+ 2·p.box_padding for i in eachindex(sizes)]
fixed   = obstacle pseudo-nodes ∪ pinned labels            # BitVector over [labels; obstacles]

if init_state === nothing            # FRESH
    seed    = initial_offsets(anchors, sizes, voronoi_cells(anchors, bounds), p;
                              pin_mask, pinned_offsets)
    offsets = side_select(anchors, sizes, psizes, bounds, seed, p;
                          pin_mask, pinned_offsets, obstacles)
    repair_crossings!(offsets, anchors, sizes, falses(n), p;
                      min_len = p.min_segment_length, pin_mask)   # discrete swap pass
else                                  # RELAX / warm-start
    offsets = copy(init_state)
end

dropped = falses(n)
# drop loop: legalize, and if infeasible drop the most-overlapped active label and retry.
# Working arrays passed to legalize = [active labels; obstacle pseudo-nodes]. A dropped
# label is EXCLUDED from constraint generation entirely (render-suppressed → blocks nothing);
# pinned labels and obstacle pseudo-nodes are INCLUDED but flagged fixed (block, never move).
while true
    lz = legalize over (active labels ∪ obstacle pseudo-nodes);
                  fixed = pinned-labels ∪ obstacle-pseudo-nodes,
                  only_move = p.only_move)
    offsets = lz.offsets    # written back to active-label slots; dropped slots keep prior offset
    (lz.residual ≤ ε || count(!, dropped) ≤ 1) && break
    drop_most_overlapped!(dropped, anchors, offsets, psizes)   # geometric, deterministic
end

if lz.residual > ε
    @warn "ProjectionSolver: residual overlap after dropping; scene over-capacity for bounds"
end

stats[] = label_cost(...) merged with (; iter = lz.rounds_used, residual = lz.residual,
                                          dropped = count(dropped))
return (; offsets, dropped, iter = lz.rounds_used, residual = lz.residual)
```

- **`drop_most_overlapped!`** — among still-active labels, drop the one whose padded box overlaps the most other active boxes; ties broken by **highest index** (deterministic). The dropped label keeps its last offset but is flagged `dropped` (render-suppressed by the recipe/connectors, exactly as today). Pinned labels are never dropped.
- **Obstacles** enter `legalize`/`side_select` as fixed pseudo-nodes carrying their `Rect2f` center and half-extents, with no leader and no eligibility for movement or dropping.
- **`pinned_offsets`** are held fixed throughout (fixed in side-select, fixed in legalize), satisfying the existing pin contract: pinned boxes still act as obstacles for the rest while their own offset never changes.
- **`@warn` on residual** mirrors `repair_crossings!`'s cap-out discipline: best-effort with a loud backstop, never silent.

**Diagnostics / `solve_stats`.** `ProjectionSolver` stores the last solve's Q plus `(iter, residual, dropped)` in `stats`. `TextRepelAlgorithm`'s existing `solve_stats(alg) -> (iter, residual)` is extended to `(; iter, residual, overlaps, mean_leader, crossings, dropped)` sourced from this Ref, giving callers the read-only Q instrument. (The two-field tuple is superseded; the stability canary test in `test_annotation_algorithm.jl` is updated to the new shape.)

## Input contract

Unchanged in spirit from v0.2; the new stages inherit the same guards:

- **NaN/Inf anchors** — filtered at `voronoi_cells` (the v0.2 finite-anchor pre-pass); the legalizer never sees them as movable constraints because their seed/side-select offsets are degenerate but bounded. No panic; diagnostics out of scope, matching v0.1/v0.2 posture.
- **Coincident anchors** — Voronoi dedup gives both `nothing` cells → TR seed; side-select + legalize separate them deterministically (legalizer's aligned-pair handling pushes coincident boxes apart along the cheaper axis).
- **`n = 0`** — short-circuit, empty result.
- **`n = 1`** — single label, no overlaps, legalizer is a no-op, `dropped = falses(1)`.
- **Over-capacity** — legalizer caps out with `residual > 0`; drop loop sheds most-overlapped labels until feasible or one label remains; `@warn` if still residual. This is the *only* drop trigger (see Backward compatibility on `max_overlaps`).

## Reactivity

The recipe's single `lift` (`src/recipe.jl`) is unchanged in dependency set — the new pipeline is pure over the same observables the v0.2 pipeline consumed (`px_anchors, text, fontsize, font, force*, only_move, box_padding, point_padding, max_overlaps, min_segment_length, bounds_obs`). Swapping `ForceSolver(params)` → `ProjectionSolver(params)` at the call site does not add or remove any reactive input. `force`, `force_point`, `force_pull` no longer influence `ProjectionSolver` output (it has no force loop), but they remain valid attributes consumed by the lift's dependency tracking and by `ForceSolver`; they become inert tuning knobs under the default solver (documented).

## Backward compatibility

- **No new public attributes.** Every v0.2 `textrepel!` / `TextRepelAlgorithm` call compiles and runs unchanged.
- **Output offsets WILL differ** from v0.2 (force loop → side-select + legalize). The change is strictly toward better layouts: zero overlap, shorter leaders, equal-or-fewer crossings. This is the same kind of intended-output-change the v0.2 design already established.
- **`force`, `force_point`, `force_pull`** become inert under the default solver (no force loop). They remain documented attributes (still consumed by `ForceSolver`, still tracked by the lift) but no longer move labels by default. Documented in the recipe docstring and CHANGELOG.
- **`max_overlaps` semantics change.** Under `ForceSolver`, `max_overlaps` dropped labels exceeding an overlap-count threshold *during* iteration. `ProjectionSolver` reaches zero overlap whenever feasible, so the only reason to drop is geometric infeasibility — `max_overlaps` no longer drives dropping. The attribute is retained for compatibility and for `ForceSolver`, but is inert under the default solver. `TextRepelAlgorithm`'s existing warn-once guard on misused `max_overlaps` is preserved. Documented as a behavior change.
- **`solve_stats` return shape** grows from `(iter, residual)` to a NamedTuple with the Q fields. Callers destructuring the two-field tuple positionally must update; this is internal-leaning API (diagnostics) and the canary test is updated alongside.
- **Determinism tests** need rebaselining (record new expected offsets), exactly as v0.2 required.

### Release prep

- **Version bump:** `Project.toml` `0.2.0` → `0.3.0`.
- **CHANGELOG:** new `## [0.3.0]` section: default solver is now `ProjectionSolver` (side-select → legalize); zero-overlap guarantee on feasible scenes; shorter leaders; `force*`/`max_overlaps` inert under the default solver; `solve_stats` returns Q diagnostics.
- **No new dependencies.** `legalize.jl`, `side_select.jl`, `cost.jl` use only GeometryBasics (already a dep). DelaunayTriangulation (v0.2) is reused for seeding.
- **`[sources]` URL flip** (memory `makietextrepel-release-blockers`, issue #5): unchanged release-blocker, tracked separately.

## Testing strategy

```
test/
  runtests.jl                  — existing; register the three new files
  test_cost.jl                 — NEW. label_cost components vs hand-computed values
  test_legalize.jl             — NEW. zero-overlap on feasible / residual on infeasible / fixed nodes / only_move
  test_side_select.jl          — NEW. seeding, greedy fixpoint, pin/obstacle handling, determinism
  test_projection_solver.jl    — NEW. end-to-end solve_cluster: zero overlap, leader ≤ force, drop loop, warm-start
  test_solver.jl               — existing; rebaseline determinism values
  test_integration.jl          — existing; rebaseline + assert zero-overlap & no-crossing invariants under new default
  test_annotation_algorithm.jl — existing; update solve_stats shape; keep the dispatch stability canary
```

### Unit tests

**`test_cost.jl`** — `label_cost` on hand-built layouts: a known-overlapping pair counts as 1 overlap (and 0 below the 0.5px threshold); `mean_leader` equals the hand-averaged norms with dropped labels excluded; `crossings` matches a hand-constructed crossing fixture (cross-check against `find_crossings` directly).

**`test_legalize.jl`** (the load-bearing guarantee):
- Feasible scenes (the §7c fixtures: knot clusters r∈{3,8,15,25}, sparse n∈{15,22,30}, collinear) → `residual == 0` after legalize, all pairs separated under the 0.5px Q check.
- Infeasible/over-capacity scene (more padded box area than bounds) → `residual > 0` at the round cap (over-capacity signal fires).
- **Fixed nodes:** a pinned/obstacle box does not move (its center is bit-identical pre/post); movable neighbors absorb the full separation.
- **`only_move = :x`:** y-coordinates of centers are unchanged; separation attempted on x only; a vertically-stacked pair that can't separate on x reports `residual > 0`.
- **Minimum displacement:** legalizing an already-separated layout is a no-op (`rounds_used == 0`, offsets bit-identical).
- Determinism: repeated calls → bit-identical offsets.

**`test_side_select.jl`**:
- Seeding: with no conflicts, each label keeps the slot nearest its Voronoi seed.
- Greedy reduces total cost monotonically across sweeps and reaches a fixpoint within `passes`.
- A two-label head-on conflict resolves to opposite sides (overlap term dominates).
- Pinned labels are never moved; obstacles are avoided (chosen slot does not overlap an obstacle when an alternative exists).
- `only_move = :x` restricts selections to horizontal slots.
- Determinism: same inputs → same selections.

**`test_projection_solver.jl`** (end-to-end through `solve_cluster`):
- On every §7e fixture: zero overlaps (0.5px Q check) **and** `mean_leader ≤` the `ForceSolver` mean_leader on the same scene (the headline quality target).
- Crossing count ≤ `ForceSolver` on the fixtures.
- **Drop loop:** an over-capacity scene ends with `count(dropped) > 0`, the surviving labels overlap-free, and the most-overlapped labels are the ones dropped (deterministic, ties by highest index).
- **Warm-start:** `init_state !== nothing` skips side-select (offsets stay near the supplied init, only separated) — verify a pre-separated init_state comes back unchanged, and an overlapping init_state comes back separated without re-siding.
- **Pin contract:** pinned offsets are bit-identical in the output; unpinned labels treat pinned boxes as obstacles.
- Determinism: bit-identical output across runs and (smoke) across a 1- vs 4-thread BLAS setting.

### Integration invariant test (`test_integration.jl`)

Extend the existing v0.2 pipeline-invariant testset so that, under the new default solver, for each fixture (sparse / dense / mixed):

```julia
@test all(isfinite, offsets)
@test all(i -> dropped[i] || within_bounds(offsets[i], params.bounds), eachindex(offsets))
@test label_cost(px_anchors, offsets, sizes, params.bounds;
                 dropped, box_padding = params.box_padding,
                 min_segment_length = min_len).overlaps == 0      # zero-overlap guarantee
@test isempty(find_crossings(connectors))                        # crossing guarantee retained
```

The zero-overlap assertion is the new load-bearing behavioral guarantee of v0.3; the no-crossing assertion is inherited from v0.2 and must still hold after the reordered pipeline.

### Out of scope for CI

- No pixel-exact PNG regression (unchanged rationale from v0.2; too fragile). The maintainer runs `examples/readme_example.jl` before tagging to eyeball the new default output.

## Determinism contract

Same `(anchors, sizes, params)` → bit-exact same offsets, across BLAS thread counts and Julia patch versions on the same platform. Per-stage source of determinism:

| Stage | Determinism source |
|-------|---------------------|
| Voronoi seed | v0.2 mechanism (lex-sorted coords + `MersenneTwister(0)`) — reused unchanged |
| `initial_offsets` | Fixed IMHOF slot ordering, integer tiebreaks — reused unchanged |
| `side_select` | Fixed candidate order, index-ordered sweeps, strict-`<` improvement (IMHOF tiebreak), no RNG |
| `repair_crossings!` | v0.2 mechanism (lex-ordered crossings, membership-only `Set`) — reused unchanged |
| `legalize` | Index-ordered constraint generation + fixed-order Dykstra sweeps, no RNG, Float64 internal |
| drop loop | Most-overlapped with highest-index tiebreak — deterministic |
| `label_cost` | Pure, read-only, no influence on placement |

Single-threaded throughout; no BLAS-reduction-order dependencies in hot paths (pure `Vec2f`/`Float64` arithmetic).

## References

- Dwyer, Marriott & Stuckey (2007), "Fast Node Overlap Removal" — VPSC / constraint-projection legalization, the keystone this design's legalizer approximates with Dykstra. https://link.springer.com/chapter/10.1007/11618058_15
- Dykstra (1983), "An algorithm for restricted least squares regression" — the cyclic projection used by `legalize`.
- Christensen, Marks & Shieber (1995), "An Empirical Study of Algorithms for Point-Feature Label Placement", ACM TOG. https://merl.com/publications/docs/TR94-12.pdf
- Imhof (1962/1975) + Liao et al. (2024), Imhof slot preference ordering (reused via `init.jl`). https://arxiv.org/abs/2407.11996
- `docs/research/2026-05-29-label-solver-first-principles.md` — the first-principles research arc (§7a component density, §7b force-projection pressure-test, §7c legalizer validation, §7d Architecture-B failure, §7e side-selection front-end validation) this design realizes.
