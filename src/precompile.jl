# precompile.jl — PrecompileTools workload.
#
# Exercises the backend-free public compute path (text measurement + the default
# ProjectionSolver pipeline) so users don't pay first-call latency on it. The
# recipe `plot!` and `annotation!` render paths need a live Makie backend
# (CairoMakie is a test-only [extra], not a runtime dep), so they are
# intentionally NOT precompiled here — the bulk of this package's own
# compilation is the pure solver/measurement layers below.
using PrecompileTools: @setup_workload, @compile_workload

@setup_workload begin
    labels  = ["alpha", "beta", "gamma"]
    anchors = [Point2f(0, 0), Point2f(20, 10), Point2f(40, 5)]
    bounds  = Rect2f(0, 0, 200, 120)
    @compile_workload begin
        # Measurement + font-symbol resolution (the default `:regular` path).
        sizes = measure_labels(labels, :regular, 14.0)
        # Fresh placement: side-select → legalize → crossings → cost → drop.
        warm_solve(anchors, sizes, bounds)
        # Warm-start (legalize-only relax) — a distinct specialization.
        warm_solve(anchors, sizes, bounds;
                   init_state = [Vec2f(5, 5), Vec2f(-5, 5), Vec2f(0, -8)])
    end
end
