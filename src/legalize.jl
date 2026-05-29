# legalize.jl — Dykstra constraint-projection node-overlap removal. Pure, GeometryBasics-only.
#
# Per round: collect currently-overlapping pairs, assign each to its cheaper
# (smaller-penetration) axis as a 1-D separation constraint, then project onto
# those constraints with Dykstra's cyclic algorithm (the minimum-displacement
# point satisfying them — the QP VPSC solves), clamp to bounds, repeat. Fixed
# nodes contribute their extents but never move. Caps at `rounds`; a positive
# returned `residual` means the scene could not be cleared in-bounds (over-capacity).

"""
Dykstra cyclic projection onto separation constraints `pos[hi] - pos[lo] >= gap`.
`movable[k]` false pins node k (its endpoint absorbs none of a violation). A
constraint between two fixed nodes is left unsatisfied (residual). Mutates `pos`.
"""
function dykstra!(pos::Vector{Float64}, cons::Vector{Tuple{Int,Int,Float64}},
                  movable::Vector{Bool}; iters::Int = 5000, tol::Float64 = 1e-3)
    m = length(cons); m == 0 && return pos
    Ilo = zeros(m); Ihi = zeros(m)          # Dykstra per-constraint correction memory
    for _ in 1:iters
        changed = 0.0
        for k in 1:m
            lo, hi, gap = cons[k]
            ylo = pos[lo] + Ilo[k]; yhi = pos[hi] + Ihi[k]
            viol = gap - (yhi - ylo)
            if viol > 0
                ml = movable[lo]; mh = movable[hi]
                if ml && mh
                    xlo = ylo - viol/2; xhi = yhi + viol/2
                elseif ml
                    xlo = ylo - viol;   xhi = yhi
                elseif mh
                    xlo = ylo;          xhi = yhi + viol
                else
                    xlo = ylo;          xhi = yhi      # both fixed: unsatisfiable
                end
            else
                xlo = ylo; xhi = yhi
            end
            Ilo[k] = ylo - xlo; Ihi[k] = yhi - xhi
            changed = max(changed, abs(pos[lo] - xlo), abs(pos[hi] - xhi))
            pos[lo] = xlo; pos[hi] = xhi
        end
        changed < tol && break
    end
    return pos
end

"""
    legalize(anchors, offsets, psizes, bounds; fixed, only_move=:both, rounds=400)
        -> (; offsets::Vector{Vec2f}, residual::Float32, rounds_used::Int)

Move boxes the minimum distance needed so no two padded boxes overlap. `psizes`
are padded sizes; box i is centered at `anchors[i] + offsets[i]` with half-extents
`psizes[i]/2`. `fixed[i]` pins box i (contributes extents, never moves) — obstacle
pseudo-nodes are passed as fixed entries by the caller. `only_move` restricts
separation to one axis. `residual` is the max remaining >0.01px penetration after
the round cap; >0.5 means over-capacity. Offsets of un-moved labels are returned
bit-identical to the input.
"""
function legalize(anchors::Vector{Point2f}, offsets::Vector{Vec2f},
                  psizes::Vector{Vec2f}, bounds::Rect2f;
                  fixed::BitVector, only_move::Symbol = :both, rounds::Int = 400)
    n = length(anchors)
    x  = [Float64(anchors[i][1] + offsets[i][1]) for i in 1:n]
    y  = [Float64(anchors[i][2] + offsets[i][2]) for i in 1:n]
    x0 = copy(x); y0 = copy(y)              # originals, for bit-identity preservation
    hw = [Float64(psizes[i][1]) / 2 for i in 1:n]
    hh = [Float64(psizes[i][2]) / 2 for i in 1:n]
    movable = [!fixed[i] for i in 1:n]
    blo = bounds.origin; bw = bounds.widths
    xlo_b = Float64(blo[1]); ylo_b = Float64(blo[2])
    xhi_b = Float64(blo[1] + bw[1]); yhi_b = Float64(blo[2] + bw[2])

    rounds_used = rounds
    for r in 1:rounds
        # Clamp movable nodes into bounds FIRST, so a non-overlapping but
        # out-of-bounds layout is still confined before the no-constraint break.
        for i in 1:n
            movable[i] || continue
            x[i] = clamp(x[i], xlo_b + hw[i], xhi_b - hw[i])
            y[i] = clamp(y[i], ylo_b + hh[i], yhi_b - hh[i])
        end
        xcons = Tuple{Int,Int,Float64}[]
        ycons = Tuple{Int,Int,Float64}[]
        for i in 1:n, j in (i+1):n
            (!movable[i] && !movable[j]) && continue        # two fixed nodes: skip
            ox = (hw[i] + hw[j]) - abs(x[i] - x[j])
            oy = (hh[i] + hh[j]) - abs(y[i] - y[j])
            if ox > 0.01 && oy > 0.01
                usex = only_move === :x ? true :
                       only_move === :y ? false : (ox <= oy)
                if usex
                    lo, hi = x[i] <= x[j] ? (i, j) : (j, i)
                    push!(xcons, (lo, hi, hw[i] + hw[j]))
                else
                    lo, hi = y[i] <= y[j] ? (i, j) : (j, i)
                    push!(ycons, (lo, hi, hh[i] + hh[j]))
                end
            end
        end
        if isempty(xcons) && isempty(ycons)
            rounds_used = r - 1
            break
        end
        dykstra!(x, xcons, movable)
        dykstra!(y, ycons, movable)
    end
    # Final clamp: guarantee in-bounds output even if the loop hit the round cap
    # right after a dykstra push (the residual below is then measured on the clamped layout).
    # On an over-capacity (round-cap) scene this clamp may pull a box back into one it had
    # been separated from; `residual` (measured next) accounts for any overlap it reintroduces.
    for i in 1:n
        movable[i] || continue
        x[i] = clamp(x[i], xlo_b + hw[i], xhi_b - hw[i])
        y[i] = clamp(y[i], ylo_b + hh[i], yhi_b - hh[i])
    end

    # final residual (post-clamp): max >0.01px penetration still present
    finalpen = 0.0
    for i in 1:n, j in (i+1):n
        (!movable[i] && !movable[j]) && continue
        ox = (hw[i] + hw[j]) - abs(x[i] - x[j])
        oy = (hh[i] + hh[j]) - abs(y[i] - y[j])
        (ox > 0.01 && oy > 0.01) && (finalpen = max(finalpen, min(ox, oy)))
    end

    new_offsets = Vector{Vec2f}(undef, n)
    for i in 1:n
        if !movable[i] || (x[i] == x0[i] && y[i] == y0[i])
            new_offsets[i] = offsets[i]                     # fixed or truly unmoved → preserve exact value
        else
            new_offsets[i] = Vec2f(x[i] - anchors[i][1], y[i] - anchors[i][2])
        end
    end
    return (; offsets = new_offsets, residual = Float32(finalpen), rounds_used = rounds_used)
end
