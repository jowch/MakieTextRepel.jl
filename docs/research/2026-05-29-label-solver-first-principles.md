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
**even sparser** for the common case). Confirm against live `solve_cluster` (still owed).
Script: `<job tmp>/component_density.py`.

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
