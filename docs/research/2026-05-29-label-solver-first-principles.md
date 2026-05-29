# A Great Label-Placement Solver, From First Principles — Research Note

> **Status: research / thinking artifact. NOT a spec, NOT a plan, NOT a commitment.**
> No implementation has been decided. This note exists to map the problem and the
> solution space deeply enough that a good design would later be obvious — and to be
> willing to conclude "the current approach is already near the right ceiling" if the
> evidence points there. Date: 2026-05-29.

## Why this note exists

The current `textrepel!` / `TextRepelAlgorithm` solver (and Makie's `annotation!`
default) are both *decent* on easy scenes and both break the same way on hard ones:
crowd points together and labels land on the wrong side, leaders grow long, crossings
appear. The goal here is **not** to be competitive with ggrepel/adjustText, and **not**
to patch the force loop. It is to ask: *if we designed a placement engine for quality
and capability from first principles, what would it be, and what could it do that
nothing currently does?* We are not beholden to porting prior art.

This note synthesizes two research passes:
1. A **literature survey** of point-feature label placement (PFLP) prior art.
2. A **12-scout fan-out** (11 returned) across direct leads *and* analogous problems
   with the same deep structure (VLSI placement, motion planning, packing, graph
   drawing, deterministic combinatorial methods), followed by a cross-domain synthesis.

## Recommendation (post-pressure-test, 2026-05-29)

The empirical work (§7a–§7d) collapsed the three candidate architectures (§4) into **one
pipeline**, not a choice between them:

- **A (constraint-projection legalize) is the validated keystone** — the only approach *proven*
  (§7c) to deliver zero overlap, at ≤1% leader cost.
- **B is not a standalone solver — it is a front-end.** Standalone discrete-exact fails on all
  three premises (§7d: discreteness ceiling, decomposition dilemma, composition invalidity). Its
  salvageable core is discrete *side-selection* (which quadrant each label takes — the right tool
  for the wrong-side errors that motivated this whole investigation).
- **C is not a competitor — it is the instrument layer.** Its cost functional Q is the scalar the
  rest of the pipeline serves and reports; it does not replace A.

**What to build, ordered by how well it is de-risked:**

| Step | What | Confidence | Effort | Determinism |
|---|---|---|---|---|
| 1 | **Read-only Q cost functional** reported via `solve_stats` (idea #3) — the measuring stick + introspection seed; commits nothing about the solver | total | low | rng-free |
| 2 | **VPSC-class constraint-projection legalizer** as a placement phase, behind the `bounds !== nothing` opt-in (idea #1) — guarantees zero overlap at minimum displacement *from its input* | high (prototyped §7c) | medium | rng-free |
| 3 | **Priority-aware drop** coupled to legalization (idea #5) — over-capacity scenes can't be legalized in-bounds (§7b/§7d), so drop/shrink lowest-priority instead | high | low | rng-free |
| 4 | **Discrete side-selection front-end** (B's salvage) — choose each label's Imhof slot via deterministic greedy/local-search, *then* legalize. **The leader-length win** (§7e: ~50% shorter leaders vs legalizing the force output, equal zero-overlap). Largely subsumes the force loop | **high (validated §7e)** | medium | rng-free |

**The winning pipeline is `discrete side-selection → legalize` (steps 4+2)**, with the read-only Q
functional (step 1) scoring it and priority-drop (step 3) handling over-capacity. §7e showed the
legalizer minimizes displacement *from its input*, so the front-end owns leader quality: legalizing
the *force* output keeps the force solver's long leaders, while side-selection → legalize halves
them at equal (zero) overlap. This pipeline **largely subsumes the continuous force loop** — discrete
slot choice + constraint projection replace it, shorter and guaranteed-clean, all rng-free.
Implementation note for step 2: use the real scan-line VPSC (corrected 2007), not the Dykstra
stand-in of §7c/§7e (the stand-in only proves the result is robust). Remaining refinement for step 4:
a crossing-aware side-search (or a retained crossing-repair pass) to fix the two minor crossing
regressions in §7e.

**Defer:** scene-bitmap awareness (idea #4), temporal coherence (#6), C's full SGD backend (the
only seeded-not-rng-free piece), and the ILP tier (needed only for rare dense/over-capacity scenes
that should drop anyway).

**Prior changed by the research:** the discrete optimizer was the expected big lever; the evidence
refined that to a two-part answer — the *legalizer* (A) guarantees separation, and the *discrete
side-selection front-end* (B's salvage) owns leader quality (§7e: ~50% shorter leaders). Together
they form a `side-select → legalize` pipeline that subsumes the continuous force loop. The
determinism tension (SA wins but needs RNG) **dissolved** — constraint projection and deterministic
side-search are both rng-free, so we never need SA. **Standalone discrete-exact (B) and standalone
force (current) are both superseded; standalone C (SGD backend) is deferred.**

---

## 1. The problem, stated cleanly

Given N anchor points in 2D, each with a text label of known w×h, choose a position
(offset) per label and draw a leader, so the figure is **readable**: no label–label
overlap, labels avoid anchors and other scene content, leaders are short and
unambiguous and don't cross, and placement respects reading conventions.

The deep structure: this is **two interleaved problems**.

- **Discrete / combinatorial** — *which side/slot* each label takes; *which* labels to
  drop. This is the NP-hard core (Marks & Shieber 1991).
- **Continuous** — the *exact* position of each box given the discrete choices.

The real objective is perceptual (readability / unambiguous attribution); overlap,
leader length, and crossings are *proxies* for it.

## 2. Diagnosis: why the current solvers hit a ceiling

- They are **optimizer-centric** ("run forces until settled"), not **objective-centric**.
  There is no explicit cost functional — just `overlap_push` / `point_push` / `force_pull`
  forces — so the solver cannot make a *principled* tradeoff (e.g. "one catastrophic
  overlap is worse than ten mild preference violations").
- They are **single-phase**. There is no projection / legalization step that *guarantees*
  zero overlap — the output is "wherever the forces happened to converge," capped by
  `max_iter`, with no convergence certificate.
- The **discrete choice is one-shot init** (Voronoi + Imhof slot in `init.jl`), never
  re-optimized. A label seeded on the wrong side sits in the wrong basin; the anchor's own
  `point_push` is an energy barrier the continuous loop won't cross. This is precisely the
  wrong-side failure we see.
- They are **scene-blind** beyond `Vector{Rect2f}` obstacles — a label will happily sit on
  top of a regression line or a dense region.
- **Dropping** is a crude unweighted `max_overlaps` count with no notion of label importance.

The literature is clear that the **quality ceiling is set by the discrete structure**, not
the continuous stepping: Christensen-Marks-Shieber 1995's dominance staircase puts
*both* continuous gradient descent *and* deterministic discrete k-opt local search
strictly below simulated annealing — and ggrepel/adjustText (pure continuous force
iteration) never reconsider a side, while D3-Labeler (the one that explores discrete
slots) uses SA. Our hypothesis — "weird placements are a discrete failure a better
continuous solver won't fix" — is **substantially supported, not proven**.

### The determinism tension

The dominant method (SA over discrete slots) needs an RNG, which collides with this
package's determinism preference. The deterministic ladder:

| Method | Determinism | Quality (per literature) |
|---|---|---|
| Pure continuous force iteration (current) | rng-free | below discrete local search |
| Deterministic k-opt / best-improvement local search | rng-free | above force, **below SA** |
| Tabu search (best-improvement + memory) | semi-deterministic, can be rng-free | matches/beats SA in one study (Yamamoto 2002) |
| Simulated annealing | needs RNG | the ceiling |
| VPSC / PRISM projection | rng-free | not a slot-chooser; *guarantees* zero overlap |
| **Exact (branch-and-bound / ILP) on small components** | **rng-free** | **optimal** |

The key reframe (below) **dissolves** this tension: the hard cases are *local and small*,
and small instances are *exactly* solvable — optimal **and** rng-free, beating SA where it
matters without any randomness.

## 3. Cross-domain convergence (the strongest signals)

Where multiple *independent* fields land on the same idea, that's the strongest signal of
what a great solver should do. The eight convergences the synthesis surfaced:

1. **Two-phase "global-smooth-then-legalize" is the #1 convergence.** VLSI (SimPL/ePlace),
   constraint layout (IPSep-CoLa), motion planning (CHOMP/TrajOpt), and graph drawing
   (stress-majorization + VPSC) *all* independently do: (a) optimize a smooth surrogate
   ignoring hard non-overlap, (b) project to the nearest overlap-free configuration,
   (c) feed the projected positions back as soft anchors and iterate. Our solver has no
   projection and no certificate. **Highest-value architectural change.**
2. **Discrete-then-continuous decomposition is universal.** Exact methods (Imhof slots =
   max-weight independent set on a conflict graph), motion planning (homotopy classes =
   slot assignments), constraint layout (Cassowary runs only *after* a side is chosen),
   graph drawing (alternating min over slots vs coordinates), packing (Sequence-Pair/B*-tree
   search the discrete arrangement) — all split discrete choice from continuous legalization.
   Our Voronoi+Imhof init already *does* the discrete phase as a one-shot seed; the
   convergence says it should be a true (re-)optimization layer.
3. **Connected-component decomposition makes exact solving essentially free for sparse
   scenes.** The N-label problem decomposes into many tiny independent sub-problems whose
   labels don't geometrically interact. Each component of size ≤8 brute-forces (256 cases);
   ≤30 solves by ILP in <10ms; larger by POPMUSIC windows. Same decomposition is the natural
   unit for parallelism, per-component optimality certificates, and temporal warm-starting.
4. **Overlap penalty must be super-linear and dominate everything.** Knuth-Plass (squared
   demerits), Hirsch (w_overlap/w_slot ≥ 50), graph drawing (log-barrier repulsion) all
   agree: one catastrophic overlap must cost more than many soft-preference violations. The
   current linear `overlap_push` with no cost functional can't express this.
5. **Bitmap/raster occupancy + summed-area-table is the universal scene-awareness primitive.**
   Scene-avoidance (occupancy bitmap), packing (d3-cloud spiral on a 1-bit bitmap), motion
   planning (signed distance field), VLSI (bin-density grid) all replace per-object geometry
   tests with render-once-query-many raster: any geometry → pixels; morphological dilation
   gives keep-out margins; a SAT answers any rectangle query in 4 lookups.
6. **Warm-start + inertia/damping is the universal temporal-coherence recipe.** Temporal
   labeling, VLSI (SimPL anchor pseudonets), motion planning (memory-of-motion, 50–60%
   fewer iterations), packing (EdWordle), warm-started tabu — all reuse the previous
   solution + a soft pull toward prior positions. We warm-start in the annotation path but
   have no inertia term and no hysteresis/rate-limiting.
7. **Priority-weighted dropping should be an objective term, not a post-hoc heuristic.**
   Quality metrics (drop_cost = priority × weight), priority degradation (importance-weighted
   MWIS; fitting cascade *before* drop), exact methods (zero-weight labels drop first) all
   encode priority directly in the selection objective.
8. **Deterministic local-minima escape exists and needs no RNG.** Motion planning
   (navigation functions / harmonic potentials are provably single-minimum; deterministic
   annealing; virtual-obstacle injection) and graph drawing (SGD schedule beats majorization
   by escaping minima) show the force-balance traps are an artifact of a bad potential, not
   fundamental. Low-discrepancy init (golden-angle — already in `init.jl`; Halton/Sobol) is
   the rng-free way to seed multiple basins.

## 3a. Leader routing & ambiguity (pillar backfilled by hand, 2026-05-29)

The leader is *half* the readability story, and the one our solver treats most crudely
(post-hoc 2-opt that only removes crossings). The literature here is mature and almost
entirely **deterministic**.

- **Leader-type taxonomy** (boundary labeling; Bekos, Kaufmann, Symvonis, Wolff and
  collaborators): leaders are classified as **straight (s)**, **orthogonal/rectilinear
  (po / opo, 1–2 bends)**, and **octilinear (o)**. We use straight leaders; the taxonomy
  matters because *type materially changes readability* (next point).
- **Readability is empirically grounded.** Barth, Gemsa, Niedermann, Nöllenburg — *On the
  Readability of Boundary Labeling* (GD 2015; full version *Information Visualization*
  18(1):110–132, 2019) — the first formal user study of leader readability. Tested four
  types (straight, L-shaped/orthogonal, diagonal, S-shaped); measured how fast/accurately a
  viewer assigns a feature to its label. **L-shaped is the best compromise** of task
  performance and preference. The algorithmic literature *uniformly forbids or minimizes
  crossings* as the primary criterion, then minimizes total length and inter-label distance;
  secondary syntheses report **crossings hurt readability more than total length** (flag:
  this specific ranking is from secondary sources / the quality-metrics scout, not confirmed
  in the full text here — verify before citing as hard).
- **Ambiguity has a quantitative handle: angular separation.** A leader is unambiguous when
  it clearly points to *one* anchor; adjacent leaders at near-parallel angles are the failure
  mode. Min pairwise angular separation between nearby leaders is the computable ambiguity
  metric (cartographic literature; surfaced by the quality-metrics scout). No single
  universally-agreed threshold was pinned down — a deep-dive into the Barth full text is owed
  for a number.
- **Crossing minimization is an ORDERING problem, separable from box placement.** Backbone
  result (Bekos, Cornelsen, Fink et al., *Many-to-One Boundary Labeling with Backbones*,
  arXiv:1308.6801, JGAA): minimizing leader crossings is **poly-time for fixed label order
  but NP-hard for flexible order**. This reframes crossing reduction as a discrete *ordering*
  search — which is exactly what our existing `crossings.jl` 2-opt swap pass already is. The
  insight: leader routing/ordering can be a clean *final phase* decoupled from where boxes sit.
- **Backbones / bundling for dense clusters.** When many labels point at one dense region, a
  per-label (or per-group) horizontal **backbone** with short feature stubs cuts clutter and
  ambiguity. Directly relevant to our knot clusters — an alternative to N separate diverging
  leaders.
- **Our setting is interior/free labeling, not boundary.** The *External Labeling Techniques:
  A Taxonomy and Survey* STAR (Bekos, Niedermann, Nöllenburg, CGF 38(3):833–860, 2019;
  arXiv:1902.01454) organizes the design space by label-position model (1-/2-/4-sided
  boundary, or free), leader shape, and reference feature, and distinguishes **internal**
  (labels near features, our case) from **external** (pushed to a boundary). We can borrow the
  leader-type + crossing-order theory, but **not** the boundary port-assignment machinery
  wholesale — our labels float near anchors rather than docking on a frame.

**Measurable leader-quality metric (the takeaway):** a deterministic, computable leader score
= w_cross · (#crossings) + w_len · (Σ leader length, normalized) + w_ang · (penalty for
adjacent leaders below an angular-separation threshold) [+ w_bend · bends, if we ever leave
straight leaders], with **w_cross the dominant weight**. This folds directly into the cost
functional Q (idea #3) as its leader terms — leader quality and box quality share one scalar.

## 4. Candidate architectures

Three sketched designs from the synthesis. These are **options to think with**, not a choice.

### A. Cost-Driven Two-Phase Solver (Relax → Legalize → Reanchor)
- **Phase 0 scene raster**: rasterize non-label geometry into an occupancy bitmap + SAT at
  viewport resolution; dilate by label half-extent for keep-out.
- **Phase 1 discrete seed**: keep the existing Voronoi+Imhof assignment.
- **Phase 2 relax**: run the existing force loop, but with overlap as a *smooth penalty* and
  a defined cost functional Q (squared overlap + leader length + slot rank + SAT scene cost).
- **Phase 3 legalize**: VPSC-style two-pass (x then y) active-set 1D QP via scan-line —
  provably overlap-free, minimum displacement; obstacles + pinned labels are the "fixed" set.
- **Phase 4 reanchor**: SimPL-style spring from each label to its legalized position with
  increasing weight α; loop 2–4 until gap Q(relaxed)−Q(legalized) < tol. **Report the gap as
  the optimality certificate.** Existing 2-opt crossing repair runs once at the end.
- *Draws from*: VLSI (SimPL, Abacus/Tetris legalization), VPSC, stress majorization, scene
  bitmap+SAT, Knuth-Plass squared penalty.
- *Strengths*: hard overlap-free **guarantee** + a true **certificate** for introspection;
  fully rng-free; largely an *evolution* of existing code (force loop → relax; `init.jl` and
  `crossings.jl` stay); uniform scene-awareness; warm-start extends to temporal coherence.
- *Risks*: must use the **corrected 2007** `satisfy_VPSC` (2005 has a feasibility bug);
  2D-via-two-1D-passes isn't globally 2D-optimal; relax-legalize loop adds iterations (cap
  it); the `bounds===nothing` byte-identity contract needs a new *opt-in* path; bitmap
  resolution is a memory/accuracy knob.

### B. Decompose-and-Solve-Exactly (component ILP / brute-force cascade)
- Enumerate feasible slots per label (≤8, pruned by bitmap+bounds) → build (label,slot)
  conflict graph (obstacles as fixed nodes) → extract connected components in O(V+E) →
  **tiered exact solve by component size**: size 1 trivial; 2 slots/label → 2-SAT via SCC
  (rng-free, O(V+E)); ≤8 → enumerate 2^k; 9–30 → JuMP+HiGHS MWIS ILP with priority weights;
  >30 → POPMUSIC rolling window (r≈15). Then continuous polish (tiny force/VPSC) within each
  chosen slot. **Report per-component MIP gap / LP dual bound** as the certificate; drop =
  low/zero-priority labels the ILP excludes.
- *Draws from*: exact methods (MWIS ILP, 2-SAT, component decomposition, POPMUSIC, JuMP+HiGHS),
  combinatorial methods, priority degradation, packing (Korf exact small-N), VLSI bisection.
- *Strengths*: **provable optimality on small local sub-problems** — exactly the project's
  value; per-component certificate is the gold standard for introspection; priority-aware
  dropping is one objective weight; naturally parallel; tractable after decomposition.
- *Risks*: JuMP+HiGHS is a **heavy dependency** (precompile, binary size) — though 2-SAT /
  brute-force tiers cover most real components without it (ConstraintSolver.jl is a pure-Julia
  alternative for N≤30); discrete slots give up the continuous-slider quality gain unless
  polish recovers it; ILP determinism needs fixed threads/seed + canonical tie-breaking;
  larger rewrite than A.

### C. Unified Multi-Criteria Functional, pluggable backend ((SGD)²-style)
- One functional Q = w_overlap·overlap² + w_leader·smooth_len + w_slot·slot_rank +
  w_cross·farkas_crossing + w_scene·SAT + w_drop·priority·(1−placed) + w_stable·|Δoffset|,
  each term normalized to [0,1] with an explicit borderline-unacceptable threshold. Minimize
  with the Zheng-2019 SGD schedule; project onto bounds each step. Crossings handled **in-loop**
  via the SPX Farkas-LP penalty (zero for non-crossing, positive otherwise), replacing the
  post-hoc 2-opt. Hosts as a second backend under the existing `AbstractClusterSolver` seam;
  `solve_stats` reports each Q sub-term (Pareto introspection).
- *Strengths*: cleanest mapping to the introspection value — the *same* scalar that guides the
  solver is reported per-criterion ("too many overlaps → raise force; too many drops → shrink
  font"); replaces the heterogeneous force knobs with one interpretable weight vector; fits the
  existing seam.
- *Risks*: SGD with pair sampling is *seeded-reproducible, not rng-free* (a determinism
  downgrade unless pair order is a fixed permutation → then it's coordinate descent, losing the
  minima-escape benefit); no hard overlap **guarantee** (still wants a VPSC backstop — so it's
  *complementary* to A, not standalone); weight tuning burden; Farkas LP per pair scales poorly
  without component decomposition.

**Note the architectures compose.** A is the conservative evolution; B is the optimality play;
C is the introspection play. A+B (legalize *and* decompose-exact) and A+C (cost-driven relax
with a VPSC backstop) are natural hybrids.

## 5. Ranked ideas (impact × effort × determinism)

| # | Idea | Impact | Effort | Determinism | Source domains |
|---|---|---|---|---|---|
| 1 | **VPSC legalization pass** (corrected 2007; scan-line + two 1D active-set QPs) as a final overlap-removal phase; obstacles + pinned = fixed set | high | med | rng-free | VPSC, VLSI legalization, IPSep-CoLa |
| 2 | **Conflict-graph + connected-component decomposition**; solve tiny components exactly (2-SAT via Graphs.jl SCC; brute-force 2^k, k≤8) before/instead of the global loop; per-component optimality | high | med | rng-free | exact methods, combinatorial, VLSI |
| 3 | **Read-only cost functional Q** (squared overlap dominant + leader len + slot rank + crossings + scene + drop), reported via `solve_stats` | high | **low** | rng-free | quality metrics, (SGD)² |
| 4 | **Occupancy bitmap + SAT** replacing `Vector{Rect2f}`; morphological dilation for keep-out; O(1) scene query | high | med | rng-free | scene-avoidance, d3-cloud, SDF |
| 5 | **Per-label priority weight** → weighted drop (fitting cascade: shrink/abbreviate before drop) + MWIS/ILP objective weight | high | **low** | rng-free | priority degradation, quality metrics |
| 6 | **Temporal inertia term** + rate-limited re-solve, warm-started from previous offsets | med | low | rng-free | temporal coherence, SimPL, memory-of-motion |
| 7 | **In-loop SPX Farkas crossing penalty** replacing post-hoc 2-opt | med | high | seeded-repro | graph drawing, Barth studies |
| 8 | **SimPL relax-legalize-reanchor outer loop** with increasing α + gap certificate | med | high | rng-free | VLSI (SimPL), stress majorization |
| 9 | **Replace `IMHOF_ORDER` with PerceptPPO ordering** (N > NE≈NW > E > SE > W > SW > S; Wallinger 2024, ~800 users) | med | **low** | rng-free | quality metrics (PerceptPPO) |
| 10 | **Angular-separation penalty** between leaders (< ~10–15° = ambiguous) — first quantitative leader-ambiguity handle | med | med | rng-free | quality metrics (Barth) |
| 11 | **Dispersion penalty** (no one-sided pile-ups) + optional coverage penalty | low | med | rng-free | Rylov-Reimer, max-entropy |
| 12 | **Deterministic minima escape**: detect frozen label (move<ε for k iters) → inject temporary virtual repulsion bump | low | low | rng-free | motion planning (virtual obstacle) |

The low-effort/high-impact cluster — **#3 (Q functional)**, **#5 (priority)**, **#9 (PerceptPPO)** —
is the cheapest place to start regardless of which architecture wins, because all three are
*measurement/parameter* changes that don't commit the solver design.

## 6. Open questions (what we don't yet know)

1. **Are real MakieTextRepel scenes sparse (many tiny conflict components) or dense (few big
   ones)?** Decides whether decompose-and-solve-exactly is essentially-free or whether the
   large-component ILP/POPMUSIC fallback dominates. *Needs measurement on real datasets.*
2. **Is a hard overlap-free guarantee actually desired**, or is ggrepel-style "mostly
   non-overlapping with graceful drops" the right product behavior? VPSC may push labels far
   from anchors (longer leaders) to reach zero overlap — the lesser evil per the metrics, but
   it changes the visual character.
3. **What determinism stance for SGD-with-pair-sampling?** Fixed pair-order → rng-free
   coordinate descent but loses the minima-escape benefit. Is seeded-reproducible acceptable
   for a v0.3 export, or must it be strictly rng-free?
4. **Is JuMP+HiGHS an acceptable dependency** given precompile latency and the current
   zero-heavy-deps posture? 2-SAT/brute-force avoids it for small components; could
   ConstraintSolver.jl (pure Julia) cover N≤30?
5. **How to source the scene bitmap from Makie without breaking the render-free guarantee
   (issue #6)?** Rasterizing geometry needs *some* render path; can Makie expose per-plot
   footprints cheaply, or does this require a backend?
6. **Does the continuous-vs-discrete quality gap** (discrete slots leave ~50% more labels
   unplaceable than a slider model) matter at our label counts, or does continuous polish
   already capture the slider benefit?
7. **What default importance metric** when the user supplies no priority? Discrete isolation
   (distance to nearest larger-attribute point) is principled but assumes an orderable attribute.

## 7. Recommended next deep-dives (concrete, ordered)

1. **Prototype the VPSC two-pass legalizer** (corrected 2007) in pure Julia; benchmark as a
   post-force projection on existing fixtures — overlap-elimination rate, total displacement,
   added leader length vs force-only. Highest-confidence, medium-effort; de-risks Architecture A.
2. **Instrument the current solver to measure conflict-graph component-size distribution &
   density** on the README example + fixtures. Empirically settles open question #1 (and #4).
3. **Build the read-only Q functional** as a measurement-only module with normalized sub-terms;
   run on current outputs; validate literature weight ratios (w_overlap/w_slot ≥ 50,
   w_crossing > w_leader). Low effort, immediately useful, becomes the objective later.
4. **Investigate scene-bitmap sourcing from Makie** without a full render / contract violation;
   survey plot-footprint / boundingbox APIs; is a CairoMakie-only fast path acceptable?
5. **Spike a component MWIS solver** with JuMP+HiGHS *and* ConstraintSolver.jl on synthetic
   dense clusters (N=10..30); compare solve time, precompile cost, determinism. Decides #4.
6. **Check PerceptPPO vs IMHOF_ORDER** on the fixtures — near-zero-effort, user-study mandate;
   confirm no crossing/overlap regression before flipping the constant.

## 7a. Findings log — Deep-dive #2 first cut (2026-05-29)

Ran a first-cut **geometric** measurement of the conflict-graph component-size
distribution (real `init.jl` slot offsets + real `box_padding`, approximated pixel label
sizes, viewport ~380×340 px, fontsize 15) across knot / uniform / collinear / sparse
scenario families. Bracketed by two coupling definitions:

- **LOOSE (provable independence):** couple i,j if *any* of the 8×8 candidate-slot pairs
  could overlap. Even the sparse control collapses into components of 12–26 — each label's
  8-slot footprint (~124×54 px) chains transitively. **Provable-independence decomposition
  does NOT make exact solving free.**
- **INIT-CONFLICT (actual first-choice overlaps):** the graph an iterative
  decompose-then-solve starts from. Knot clusters (README-like, n=22) → max component
  **5–10**; sparse n≤30 → ≤19; uniform n=40 → 6 (all brute-forceable, no ILP). Breaks only
  in dense regimes: uniform n=80 → max **62**; n=150 / tight collinear → one giant (ILP needed).

**Implications:**
1. Open question #1: typical scenes (knots + scatter) **are** sparse under the actual-conflict
   graph — components stay ≤~10, brute-force-trivial, rng-free. Architecture B's "exact is
   essentially free" holds *for the common case*.
2. Open question #4: JuMP+HiGHS is needed **only** for dense regimes (packed fields ≳80, tight
   collinear) → it can be a **lazy/optional** dependency behind a brute-force threshold.
3. Methodological crux the synthesis glossed: decomposition viability hinges on a *smart*
   coupling (candidate-slot pruning, or iterative decompose-on-actual-conflicts). Provable
   independence is too coarse to exploit.

**Caveats:** geometric model, approximated label sizes, **TR-only init** (real `init.jl` picks
best-fit Voronoi+Imhof, which *spreads* initial placement → real init-conflict graph likely
**even sparser** for the common case).
Script: `<job tmp>/component_density.py`.

**Live confirmation (same day, `<job tmp>/component_live.jl`, real `solve_cluster`).** Ran the
real `voronoi_cells` + `initial_offsets` + full solve on the same scenarios. Results **confirm
the first cut** and, as predicted, the real Voronoi+Imhof init is *slightly sparser* than the
TR-only model:
- Real-init max component — knots: **4–8**; sparse n≤30: **4–13**; uniform n=40: **6**
  (all brute-forceable, no ILP). Dense only: uniform n=80 → **62**, n=150 / tight-collinear →
  one giant. → **Decompose-and-solve-exactly is viable for the common case; ILP fallback needed
  only in dense regimes.** Open questions #1 and #4 answered.
- **Bonus finding (supports ranked idea #1).** Post-solve the force loop leaves substantial
  **residual padded-box overlaps** with `max_overlaps=Inf` dropping nothing: ~8–11 overlapping
  pairs even on easy knots; 32 pairs / max-residual-component 21 on uniform n=40; 127 on n=80.
  Direct empirical evidence the current solver **does not guarantee separation** — a VPSC-style
  legalization pass (idea #1) has concrete, measurable value even on easy scenes. (Caveat:
  "overlap" here = padded-box intersection, i.e. labels within ~2·box_padding=8px of clear
  space, so it includes near-touching as well as true glyph overlap.)

## 7b. Architecture A pressure-test (2026-05-29, `<job tmp>/component_pressure.jl`)

Stress-tested Architecture A's central risk (open question #2): does a zero-overlap
**legalization** projection blow up leader length? Probe: take `solve_cluster`'s output, run a
pure separation projection (only `overlap_push`, full steps to convergence, in-bounds),
measure leader-length cost. Three findings, the first unexpected:

1. **A force-style projection does NOT guarantee zero overlap.** Even run to convergence with
   no competing anchor-pull, the separation projection left residual overlaps in nearly every
   crowded case (knots 3–6, uniform n=40 → 19, n=80 → 111, collinear → 5–10). It hits the
   *same* force-balance / local-minimum trap as the main solver — a label squeezed between two
   others receives cancelling pushes and "converges" with overlaps intact. **Implication:
   Architecture A's Phase 3 legalize CANNOT be a dressed-up force loop — it must be the genuine
   VPSC active-set constraint QP.** The zero-overlap guarantee is real but is *only* delivered
   by the actual QP machinery, never by "iterate the separation forces a bit more." This rules
   out the cheap shortcut and **escalates deep-dive #1 (prototype real VPSC) to the critical
   path** — we've now empirically shown the easy substitute fails.
2. **For common scenes, legalization's leader cost is LOW** — open question #2's fear does not
   materialize. Knots / sparse / moderate: leader-mean change −6% to +8%, max displacement
   ≤~60 px. Since a force-style projection over-displaces vs the minimum-displacement VPSC QP,
   these are *pessimistic upper bounds* — real VPSC would be equal or better. Good news for A.
3. **For over-capacity dense scenes, zero-overlap is physically impossible in-bounds → you must
   DROP.** Uniform n=80 (80 labels in 380×340) could not be separated: forcing it flung labels
   a max of **287 px** and inflated mean leader length **+153%**, and *still* left 111 overlaps.
   This couples legalization to **priority-aware dropping (idea #5)**: you cannot legalize an
   over-full scene; drop/shrink first, then legalize the remainder. Architecture A needs idea #5
   baked in, not bolted on.

**Verdict:** Architecture A is not broken, but the pressure-test sharpened its non-negotiables —
(a) Phase 3 must be true VPSC, not force iteration; (b) it must be coupled with priority-dropping
for over-capacity scenes; (c) the leader-length risk is low for common scenes and only bites in
the over-capacity regime, where the correct response is to drop rather than separate. Next
critical experiment: **prototype the real VPSC QP** (deep-dive #1) and re-run this probe with it.

*Caveat:* the probe's "legalize" is force-style, not the real minimum-displacement VPSC; finding
1 is about the *shortcut* failing (the insight — need the real QP — holds), and findings 2–3 are
method-robust (upper-bound costs; over-capacity is a geometric fact).

## 7c. Deep-dive #1: constraint-projection legalizer VALIDATED (2026-05-29, `<job tmp>/vpsc_legalize.jl`)

Built the real thing §7b said was needed: a **constraint-projection** legalizer (Dykstra cyclic
projection onto the separation half-spaces — the same minimum-displacement QP VPSC solves, via a
simpler-to-verify solver; per-axis constraints, cheaper-axis assignment, iterated to zero
overlap, bounds by clamping). Re-ran the §7b probe with it. The contrast is decisive:

| Scene | force-style projection (§7b) | constraint projection (this) |
|---|---|---|
| knots / sparse / collinear | 3–6 residual overlaps (FAILS) | **0 overlap**, leader +≤1%, max disp ≤7 px |
| uniform n=40 (~41% area) | 19 residual, −16% leader | **0 overlap**, ~0% leader, max disp 11 px |
| over-capacity uniform n=80 | 111 residual, +153% leader, 287 px disp | 49 residual, +39% leader, 100 px disp |

**Conclusions:**
1. **The zero-overlap guarantee is real AND essentially free for feasible scenes.** Constraint
   projection reaches 0 overlap on every fittable scene (all knots, sparse, collinear, uniform up
   to n=40) at **≤1% leader-length change and ≤11 px max displacement**. Open question #2 is
   answered: legalization does *not* blow up leaders in the common case — it makes tiny local
   adjustments to close the overlaps the force solver left.
2. **Method matters enormously.** Same scenes, same objective: force balance leaves overlaps and
   (when dense) flings labels; constraint projection guarantees separation cheaply. This confirms
   §7b finding 1 from the positive side — Architecture A's Phase 3 must be constraint projection,
   and once it is, it is a high-value, low-cost win.
3. **Over-capacity still must drop.** uniform n=80 cannot reach 0 even at 400 rounds (49 residual)
   and inflates leaders +39% when forced — but note it is already far better than the force
   probe's +153%. Confirms the legalize/drop coupling (idea #5).

**Verdict on Architecture A:** its keystone phase is validated. A VPSC-class legalizer bolted onto
the current `solve_cluster` output would eliminate the residual overlaps measured in §7a at
negligible leader cost — the single highest-confidence improvement we have evidence for.

*Caveats:* (a) the legalizer is a VPSC-*equivalent* (Dykstra projection + iterated cheaper-axis
assignment), not the exact scan-line VPSC — real VPSC would be faster (single QP/axis, O(n log n)
constraint generation) and no worse on displacement, so the qualitative result is robust/conservative;
(b) "overlap" = padded-box intersection, so "0 overlap" means ~8 px clear space between every pair;
(c) leader-mean is near-unchanged partly because the force solver's output was already nearly
separated — the win is converting "nearly, with 8–31 stragglers" into a *guaranteed* 0 at no cost.

## 7d. Architecture B pressure-test — standalone B fails (2026-05-29, `<job tmp>/brute_b.jl`)

Branch-and-bound exact solver over in-bounds Imhof slots per component, testing B's three
premises. **All three break:**

1. **Decomposition dilemma — CONFIRMED.** The *provable-independence* (loose) components —
   what exact solving actually requires — are large everywhere: knots **10–13**, sparse
   **13–24**, collinear 10–20, uniform 40–80. Full-slot enumeration is **8^k**, not 2^k:
   measured `max-combos` up to **5.5e14** (uniform n=80) and **1.7e17** (collinear n=20).
   B&B prunes small components but fell back to greedy on the large ones. The synthesis's
   "k≤8 → 256 cases" silently assumed 2 slots/label; with 8 Imhof slots it is intractable, and
   the valid-independence components do **not** stay small.
2. **Composition invalidity — CONFIRMED.** Solving the *sparse* init-conflict components
   independently and stitching leaves **cross-component overlaps**: knots 10–21, uniform n=40
   → 13, sparse → 3–9. Cleanest case — **sparse n=22: within-component residual = 0 (every
   component separated perfectly) but stitched-global = 9** overlaps, all cross-component. The
   sparse graph is not a valid independence relation; independent solving doesn't compose.
3. **Discreteness ceiling — CONFIRMED, and it's the killer.** Even at the *proven optimum*, the
   discrete 8-slot model leaves residual overlaps **within** a component: knot r=3 → **18**,
   r=8 → 9, r=15 → 9, sparse n=30 → 6. On a 3 px-radius knot, **no assignment of the 8 Imhof
   slots separates the labels** — they are too close and the fixed slots collide.

**Decisive contrast with A on the same tight knots:** A's constraint-projection legalizer
reached **0 overlap** at ≤1% leader cost (§7c); B's *provably optimal* discrete solve leaves
**9–18** overlaps. B's "provable optimality" is optimality within a model too weak to solve the
problem — you prove a placement optimal that still overlaps. Continuous positioning is what
separates tight clusters; discrete slots cannot.

**Verdict:** **standalone Architecture B is not viable** for our problem — discreteness ceiling +
decomposition dilemma + composition invalidity. But its genuine contribution survives: discrete
**side-selection** (which quadrant each label takes) is the right tool for the *discrete*
sub-problem (wrong-side errors), and belongs as a **front-end phase feeding A's continuous
legalizer**, not as a standalone solver. This is exactly cross-cutting insight #2
(discrete-then-continuous) and Architecture A's Phase 1 (discrete seed) + Phase 3 (legalize).
**The evidence converges on A with a discrete side-selection front-end — i.e., the A/B hybrid —
not on standalone B.**

*Caveats:* candidates = 8 in-bounds Imhof slots with **no continuous freedom** (that *is* B — a
discrete model; allowing slot+nudge would make it converge toward A); `within-resid` counts only
exactly-solved components (greedy-fallback components' overlaps show only in stitched-global);
"overlap" = padded-box (8 px clearance). The knot within-resid values are proven optima (all
knot components solved exactly under the node cap), so the discreteness ceiling is a hard result.

## 7e. Spike: discrete side-selection front-end VALIDATED — the leader-length win (2026-05-29, `<job tmp>/pipeline_compare.jl`)

Compared three pipelines on an aligned Q scorer (overlaps >0.5 px penetration, mean leader length,
leader crossings), 400 legalize rounds: (1) BASELINE = current `solve_cluster`; (2) FORCE+LEGAL =
legalize the baseline (A alone); (3) SIDE+LEGAL = greedy deterministic discrete slot search →
legalize (A + B's front-end).

**Result: both legalized pipelines reach 0 overlap everywhere, but SIDE+LEGAL halves leader
length in every scenario** at equal overlap and equal-or-better crossings:

| Scene | FORCE+LEGAL lead | SIDE+LEGAL lead | reduction |
|---|---|---|---|
| knot r=3 | 49.5 | 26.8 | −46% |
| knot r=25 | 44.3 | 18.1 | −59% |
| sparse n=22 | 37.1 | 12.9 | −65% |
| uniform n=20 | 17.1 | 7.5 | −56% |
| collinear n=10 | 76.1 | 15.2 | −80% |

(Crossings equal-or-better except two minor regressions: knot r=25 0→2, collinear n=20 5→6.)

**Why — the load-bearing insight:** the legalizer minimizes displacement *from its input*, so it
**preserves the leader-length character of its starting configuration**. The force solver inflates
leaders (repulsion flings labels far out) and legalization keeps them there; side-selection places
each label at its tight preferred Imhof slot and legalization only nudges to clear overlaps. **The
starting configuration determines final leader length — so the front-end, not the legalizer, owns
leader quality.**

**Consequences:**
1. **Step 4 is validated, not a gamble** — discrete side-selection → legalize is a clear, consistent
   win (~50% shorter leaders, equal zero-overlap).
2. **Reframes step 2:** merely legalizing the *force* output keeps the long leaders. The leader win
   requires *replacing* the force front-end with side-selection.
3. **The winning pipeline `side-select → legalize` largely SUBSUMES the force loop** — no continuous
   repulsion phase needed; discrete slot choice + constraint projection do the job, shorter and
   guaranteed-clean.

*Caveats:* the side-selector is a simple deterministic greedy local-search (6 passes) — a better one
could improve further and likely fix the two crossing regressions (or keep a final crossing-repair
pass); synthetic data, but the *same* aligned metric scores all three pipelines, so the comparison
is fair; "leader length" = mean |offset|, and Imhof slots keep labels adjacent-but-not-occluding via
`point_padding`.

## 8. Gaps & caveats in this research

- **The leader-routing scout failed three times** (twice on structured-output, once on a
  lone-surrogate encoding error from fetched web content that poisoned the subagent context).
  Backfilled **by hand** instead — see §3a. Confidence is good on the deterministic
  algorithmic facts (leader-type taxonomy, fixed-vs-flexible-order crossing complexity,
  backbones, the external-labeling taxonomy) but the arXiv abstract pages didn't yield
  full-text specifics; two numbers are still owed from the Barth full text: (a) confirmation
  that crossings outrank length, and (b) a concrete angular-separation ambiguity threshold.
- The PDFs of several seminal papers (CMS, VPSC, PRISM, (GD)²) indexed as binary and weren't
  text-searchable in the sandbox; specific numbers (CMS dominance staircase, VPSC complexity)
  are from cross-checked secondary syntheses, not verbatim extraction. Re-verify exact figures
  before citing them as hard claims.
- "Continuous can't help" is **too strong** — VPSC's zero-overlap guarantee is a *continuous*
  contribution no force iteration provides. The defensible claim is "continuous stepping can't
  fix the *discrete* artifacts (wrong-side, preventable crossings)."

## 9. Key references (curated)

**Quality / objective formalization**
- Christensen, Marks, Shieber 1995 — *An Empirical Study of Algorithms for PFLP* (ACM TOG) — the dominance staircase. https://www.eecs.harvard.edu/shieber/Biblio/Papers/tog-final.pdf
- Knuth & Plass 1981 — *Breaking Paragraphs into Lines* — squared-demerits global badness model.
- Barth, Gemsa, Niedermann, Nöllenburg 2015/2019 — *On the Readability of (Leaders in) Boundary Labeling* — crossings > length; angular separation = ambiguity. https://arxiv.org/abs/1509.00379
- Wallinger et al. 2024 — *PerceptPPO* — ~800-user slot-preference study (N > NE).
- Marks & Shieber 1991 — *The Computational Complexity of Cartographic Label Placement* (NP-completeness).

**Leader routing & ambiguity (§3a)**
- Bekos, Niedermann, Nöllenburg 2019 — *External Labeling Techniques: A Taxonomy and Survey* (CGF 38(3):833–860). https://arxiv.org/abs/1902.01454
- Barth, Gemsa, Niedermann, Nöllenburg 2015/2019 — *On the Readability of Boundary Labeling* (GD 2015; Information Visualization 18(1):110–132) — L-shaped best; first leader-readability user study. https://arxiv.org/abs/1509.00379
- Bekos, Cornelsen, Fink et al. 2013 — *Many-to-One Boundary Labeling with Backbones* (JGAA) — crossing min poly for fixed order, NP-hard for flexible. https://arxiv.org/abs/1308.6801

**Guaranteed separation / constraint layout**
- Dwyer, Marriott, Stuckey 2005 + 2007 correction — *Fast Node Overlap Removal* (VPSC). https://people.eng.unimelb.edu.au/pstuckey/papers/gd2005b.pdf
- Gansner & Hu 2010 — *Efficient, Proximity-Preserving Node Overlap Removal* (PRISM).
- Dwyer, Koren, Marriott 2006 — *IPSep-CoLa* (incremental separation-constraint layout). https://ieeexplore.ieee.org/document/4015435/
- Badros, Borning, Stuckey 2002 — *The Cassowary Linear Arithmetic Constraint Solving Algorithm*. https://constraints.cs.washington.edu/solvers/cassowary-tochi.pdf

**Exact / decomposition / deterministic combinatorial**
- Alvim & Taillard 2009 — *POPMUSIC for the PFLP*. https://www.sciencedirect.com/science/article/abs/pii/S0377221707010041
- Yamamoto, Câmara, Lorena 2002 — *Tabu Search Heuristic for PFLP* (deterministic-leaning, beats SA in their study).
- Dunning, Huchette, Lubin 2017 — *JuMP*. https://arxiv.org/pdf/1508.01982

**Analogous problems**
- VLSI: SimPL / RePlAce global placement; Abacus/Tetris legalization; recursive bisection. https://vlsicad.ucsd.edu/Publications/Journals/j126.pdf
- Motion planning: Khatib 1986 (potential fields); Connolly & Burns 1990 (Laplace/harmonic, single-minimum); CHOMP/TrajOpt. https://courses.cs.washington.edu/courses/cse599j/12sp/papers/Connolly.pdf
- Graph drawing: Ahmed et al. (GD)² 2020 / (SGD)² 2022; Devkota et al. SPX 2019; stress majorization (SMACOF). https://arxiv.org/abs/2008.05584 · https://arxiv.org/abs/2112.01571 · https://arxiv.org/abs/1908.01769
- Temporal: Been, Daiches, Yap 2006 — *Dynamic Map Labeling*; Barth et al. 2016/2020 — *Temporal Map Labeling*; Bobák, Čmolík, Čadík 2020/2024 (temporally stable / RL labeling). https://arxiv.org/abs/1609.06327
- Packing: d3-cloud (Wordle spiral); MAXRECTS/skyline; Korf exact rectangle packing.

*(Full 139-entry citation list from the scout fan-out is preserved in the workflow run
artifacts; this is the load-bearing curated subset.)*
