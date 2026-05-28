# stress_compare.jl — three crowding scenarios x four panels each.
#
# Panels per scenario:
#   1. annotation! (default LabelRepel)
#   2. annotation! + TextRepelAlgorithm
#   3. textrepel! (recipe, default)
#   4. textrepel! with max_overlaps = 3, background = true
#
# Run from worktree root:
#   julia --project=. examples/stress_compare.jl

using CairoMakie
using MakieTextRepel
using Random
include(joinpath(@__DIR__, "..", "src", "annotation_algorithm.jl"))

const OUTDIR = joinpath(@__DIR__, "..", "test", "output")
mkpath(OUTDIR)

# Reasonable per-panel canvas. The whole figure is 4 panels wide so it's chunky
# but each axis gets enough real-estate that label collisions are visible.
const PANEL_W = 520
const PANEL_H = 520

# Word pool — varied lengths to keep box widths heterogeneous.
const WORDS = [
    "Apple","Banana","Cherry","Date","Elderberry","Fig","Grape","Honeydew",
    "Indian Fig","Jackfruit","Kiwi","Lychee","Mango","Nectarine","Orange",
    "Papaya","Quince","Raspberry","Strawberry","Tangerine","Ugli","Voavanga",
    "Watermelon","Ximenia","Yumberry","Zucchini","Avocado","Blueberry",
    "Cranberry","Durian","Eggplant","Feijoa","Guava","Huckleberry","Imbe",
    "Jujube","Kumquat","Loquat","Medlar","Noni","Olive","Persimmon",
    "Rambutan","Salak","Tamarind","Plum","Pear","Lime","Lemon","Coconut",
]

label_pool(n) = [WORDS[mod1(i, length(WORDS))] * (i > length(WORDS) ? " $(i)" : "")
                 for i in 1:n]

function render_scenario(name, pts, labels, limits)
    fig = Figure(size = (PANEL_W * 4, PANEL_H + 60), fontsize = 13)
    Label(fig[0, 1:4], name; fontsize = 18, font = :bold, tellwidth = false)

    ax1 = Axis(fig[1, 1]; limits, title = "annotation! (default LabelRepel)")
    scatter!(ax1, pts; color = :tomato, markersize = 7)
    annotation!(ax1, pts; text = labels)

    ax2 = Axis(fig[1, 2]; limits, title = "annotation! + TextRepelAlgorithm")
    scatter!(ax2, pts; color = :tomato, markersize = 7)
    annotation!(ax2, pts; text = labels, algorithm = TextRepelAlgorithm())

    ax3 = Axis(fig[1, 3]; limits, title = "textrepel! (recipe, defaults)")
    scatter!(ax3, pts; color = :tomato, markersize = 7)
    textrepel!(ax3, pts; text = labels)

    ax4 = Axis(fig[1, 4]; limits,
               title = "textrepel! max_overlaps=3, background=true")
    scatter!(ax4, pts; color = :tomato, markersize = 7)
    textrepel!(ax4, pts; text = labels,
               max_overlaps = 3, background = true,
               backgroundcolor = (:white, 0.85), strokecolor = :gray60,
               strokewidth = 0.5)

    foreach(hidedecorations!, (ax1, ax2, ax3, ax4))

    out = joinpath(OUTDIR, "stress_$(name).png")
    save(out, fig; px_per_unit = 1.5)
    println("wrote: ", out)
    return out
end

# ── Scenario 1: ~50 labels, moderately crowded ────────────────────────────
Random.seed!(20260527)
n1 = 50
pts1 = [(randn() * 1.2, randn() * 1.0) for _ in 1:n1]
labels1 = label_pool(n1)
lim1 = (-3.5, 3.5, -3.0, 3.0)
render_scenario("01_moderate_n50", pts1, labels1, lim1)

# ── Scenario 2: ~100 labels, severely crowded ─────────────────────────────
Random.seed!(20260528)
n2 = 100
pts2 = [(randn() * 1.0, randn() * 0.8) for _ in 1:n2]
labels2 = label_pool(n2)
lim2 = (-3.0, 3.0, -2.5, 2.5)
render_scenario("02_severe_n100", pts2, labels2, lim2)

# ── Scenario 3: pathological cluster, 30 pts in a 0.05x0.05 box ───────────
Random.seed!(20260529)
n3 = 30
cx, cy = 0.5, 0.5
pts3 = [(cx + (rand() - 0.5) * 0.05, cy + (rand() - 0.5) * 0.05) for _ in 1:n3]
# Keep label lengths similar so box widths are uniform — exposes label-on-label
# collisions without width heterogeneity confounding the picture.
labels3 = ["Sample $(lpad(i, 2, '0'))" for i in 1:n3]
lim3 = (0.0, 1.0, 0.0, 1.0)
render_scenario("03_pathological_cluster_n30", pts3, labels3, lim3)

println("done.")
