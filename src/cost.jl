# cost.jl — read-only placement quality functional (Q). Pure, GeometryBasics-only.

"""
    label_cost(anchors, sizes; offsets, bounds, dropped=nothing,
               box_padding, point_padding=0.0, min_segment_length)
        -> (; overlaps::Int, point_overlaps::Int, mean_leader::Float32, crossings::Int)

Read-only quality measure of a placement. Never mutates, never feeds back into
placement. Four independent components (reported separately, never collapsed):

- `overlaps`  — count of label pairs whose padded boxes penetrate by > 0.5 px on
  both axes (the visual-overlap threshold; sub-0.5 px touches are ignored).
- `point_overlaps` — count of non-dropped labels covering a non-dropped foreign
  anchor, via the shared `point_covered`, using `point_padding`.
- `mean_leader` — mean ‖offset‖ (label displacement) over non-dropped labels
  (0 if none). Note this is offset magnitude, not the padding-trimmed rendered
  leader length.
- `crossings` — crossing leader pairs, via `connector_for` + `find_crossings`.

Labels flagged in `dropped` are excluded from `overlaps`, `point_overlaps`, and
`mean_leader` (they are render-suppressed) — a dropped label neither covers a
marker nor is counted as a covered anchor. `box_padding`/`point_padding`/`min_segment_length`
must match the recipe values so the crossing count agrees with what renders.
`bounds` is accepted for call-shape parity with the recipe/solver and is
currently unused (reserved for future viewport-aware metrics).
"""
function label_cost(anchors::Vector{Point2f}, sizes::Vector{Vec2f};
                    offsets::Vector{Vec2f}, bounds::Rect2f,
                    dropped::Union{Nothing,BitVector} = nothing,
                    box_padding::Real, point_padding::Real = 0.0,
                    min_segment_length::Real)
    n = length(anchors)
    isdrp(i) = dropped !== nothing && dropped[i]
    hw = [Float64(sizes[i][1] + 2box_padding) / 2 for i in 1:n]
    hh = [Float64(sizes[i][2] + 2box_padding) / 2 for i in 1:n]
    cx = [Float64(anchors[i][1] + offsets[i][1]) for i in 1:n]
    cy = [Float64(anchors[i][2] + offsets[i][2]) for i in 1:n]

    overlaps = 0
    for i in 1:n, j in (i+1):n
        (isdrp(i) || isdrp(j)) && continue
        ox = (hw[i] + hw[j]) - abs(cx[i] - cx[j])
        oy = (hh[i] + hh[j]) - abs(cy[i] - cy[j])
        (ox > 0.5 && oy > 0.5) && (overlaps += 1)
    end

    point_overlaps = 0
    for i in 1:n
        isdrp(i) && continue
        bm = Rect2f(Point2f(cx[i] - Float64(sizes[i][1]) / 2, cy[i] - Float64(sizes[i][2]) / 2),
                    Vec2f(sizes[i]))                              # unpadded text box of label i
        for j in 1:n
            (j == i || isdrp(j)) && continue
            point_covered(anchors[j], bm, point_padding) && (point_overlaps += 1)
        end
    end

    active = [i for i in 1:n if !isdrp(i)]
    mean_leader = isempty(active) ? 0f0 :
        Float32(sum(sqrt(Float64(offsets[i][1])^2 + Float64(offsets[i][2])^2)
                    for i in active) / length(active))

    params = RepelParams(; box_padding = box_padding, point_padding = point_padding)
    connectors = [connector_for(anchors[i], offsets[i], sizes[i], isdrp(i),
                                params, min_segment_length) for i in 1:n]
    crossings = length(find_crossings(connectors))

    return (; overlaps = overlaps, point_overlaps = point_overlaps,
              mean_leader = mean_leader, crossings = crossings)
end
