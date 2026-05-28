# Probes for Makie.annotation! edge cases.
# Goal: verify behavior under conditions of interest documented in
# docs/annotation-research/03-limitations-and-stability.md.
#
# Each block is independent. Run with:
#   julia --project=. examples/probe_annotation_edges.jl
#
# This script does NOT modify Makie sources. It only invokes the public recipe.

using CairoMakie

println("== Probe 0: package version ==")
println("Makie = ", pkgversion(Makie))

# --- Probe 1: zero labels -------------------------------------------------
println("\n== Probe 1: 0 labels ==")
try
    fig = Figure()
    ax = Axis(fig[1, 1])
    p = annotation!(ax, Float64[], Float64[]; text = String[])
    save("/tmp/probe_zero_labels.png", fig)
    println("  OK: 0 labels -> offsets length = ", length(p.offsets[]))
catch err
    println("  ERROR: ", typeof(err), ": ", sprint(showerror, err))
end

# --- Probe 2: all labels at the same point --------------------------------
println("\n== Probe 2: all labels at same point ==")
try
    fig = Figure()
    ax = Axis(fig[1, 1]; limits = (0, 2, 0, 2))
    xs = fill(1.0, 5)
    ys = fill(1.0, 5)
    p = annotation!(ax, xs, ys; text = ["A", "B", "C", "D", "E"])
    save("/tmp/probe_same_point.png", fig)
    offs = p.offsets[]
    println("  OK: offsets after solve = ", offs)
    println("  any NaN? ", any(o -> any(isnan, o), offs))
    println("  all distinct? ", length(unique(offs)) == length(offs))
catch err
    println("  ERROR: ", typeof(err), ": ", sprint(showerror, err))
end

# --- Probe 3: empty-string labels (test case but reproduce here) ---------
println("\n== Probe 3: empty strings ==")
try
    fig = Figure()
    ax = Axis(fig[1, 1])
    p = annotation!(ax, [1.0, 2.0, 3.0], [1.0, 2.0, 3.0]; text = ["A", "", "C"])
    save("/tmp/probe_empty_string.png", fig)
    println("  OK: offsets = ", p.offsets[])
    println("  any NaN? ", any(o -> any(isnan, o), p.offsets[]))
catch err
    println("  ERROR: ", typeof(err), ": ", sprint(showerror, err))
end

# --- Probe 4: LaTeXString labels -----------------------------------------
println("\n== Probe 4: LaTeX labels ==")
try
    fig = Figure()
    ax = Axis(fig[1, 1])
    p = annotation!(
        ax, [1.0, 2.0], [1.0, 2.0];
        text = [L"\alpha^2", L"\sum_{i=1}^n x_i"],
    )
    save("/tmp/probe_latex.png", fig)
    println("  OK: offsets = ", p.offsets[])
catch err
    println("  ERROR: ", typeof(err), ": ", sprint(showerror, err))
end

# --- Probe 5: rich text --------------------------------------------------
println("\n== Probe 5: rich text ==")
try
    fig = Figure()
    ax = Axis(fig[1, 1])
    p = annotation!(
        ax, [1.0, 2.0], [1.0, 2.0];
        text = [rich("bold ", rich("italic"; font = :italic)), rich("normal")],
    )
    save("/tmp/probe_rich.png", fig)
    println("  OK: offsets = ", p.offsets[])
catch err
    println("  ERROR: ", typeof(err), ": ", sprint(showerror, err))
end

# --- Probe 6: log-scale axis ---------------------------------------------
println("\n== Probe 6: log-scale axis ==")
try
    fig = Figure()
    ax = Axis(fig[1, 1]; xscale = log10, yscale = log10)
    xs = [1.0, 10.0, 100.0, 1000.0]
    ys = [1.0, 10.0, 100.0, 1000.0]
    scatter!(ax, xs, ys)
    p = annotation!(ax, xs, ys; text = ["a", "b", "c", "d"])
    save("/tmp/probe_log.png", fig)
    println("  OK: offsets = ", p.offsets[])
catch err
    println("  ERROR: ", typeof(err), ": ", sprint(showerror, err))
end

# --- Probe 7: labels overflowing viewport --------------------------------
println("\n== Probe 7: tiny viewport, big labels ==")
try
    fig = Figure(size = (200, 200))
    ax = Axis(fig[1, 1]; limits = (0, 1, 0, 1))
    xs = [0.1, 0.5, 0.9]
    ys = [0.5, 0.5, 0.5]
    scatter!(ax, xs, ys)
    p = annotation!(
        ax, xs, ys;
        text = ["a-really-long-label-A", "long-B-label", "long-C-label"],
    )
    save("/tmp/probe_overflow.png", fig)
    println("  OK: offsets = ", p.offsets[])
catch err
    println("  ERROR: ", typeof(err), ": ", sprint(showerror, err))
end

# --- Probe 8: large N (perf hint) ----------------------------------------
println("\n== Probe 8: N=500 timing (single solve) ==")
try
    fig = Figure(size = (1000, 1000))
    ax = Axis(fig[1, 1]; limits = (0, 1, 0, 1))
    N = 500
    xs = rand(N); ys = rand(N)
    t0 = time()
    p = annotation!(ax, xs, ys; text = string.(1:N), maxiter = 200)
    save("/tmp/probe_n500.png", fig)
    println("  OK: total time ", round(time() - t0; digits = 2), "s for N=", N)
catch err
    println("  ERROR: ", typeof(err), ": ", sprint(showerror, err))
end

# --- Probe 9: maxiter = 0 -------------------------------------------------
println("\n== Probe 9: maxiter = 0 (no iterations) ==")
try
    fig = Figure()
    ax = Axis(fig[1, 1])
    p = annotation!(ax, [1.0, 1.1], [1.0, 1.0]; text = ["A", "B"], maxiter = 0)
    save("/tmp/probe_maxiter0.png", fig)
    println("  OK: offsets after 0 iterations = ", p.offsets[])
catch err
    println("  ERROR: ", typeof(err), ": ", sprint(showerror, err))
end

# --- Probe 10: text-position rotation (unsupported by recipe?) -----------
println("\n== Probe 10: trying rotation kwarg ==")
try
    fig = Figure()
    ax = Axis(fig[1, 1])
    p = annotation!(
        ax, [1.0, 2.0], [1.0, 2.0];
        text = ["A", "B"],
        rotation = pi/4,
    )
    save("/tmp/probe_rotation.png", fig)
    println("  OK: rotation kwarg accepted? offsets = ", p.offsets[])
catch err
    println("  ERROR (expected — no rotation attr): ", typeof(err), ": ", sprint(showerror, err))
end

println("\nAll probes finished.")
