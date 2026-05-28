# annotation_algorithm.jl — pluggable algorithm for Makie's `annotation!` recipe.
#
# Spike — not yet wired into the package. Load ad-hoc from the spike script.
#
# Caveats vs. the standalone `textrepel!` recipe:
#   • Text bounding boxes are supplied by Makie's glyph layout (text_bbs argument),
#     NOT by TextMeasure. Plain-text sizing fidelity is whatever Makie gives.
#   • Label dropping (`max_overlaps`) has no home in annotation's data model and
#     is not exposed here.
#   • Wrapper assumes annotation's default center alignment — text_bbs whose
#     origin is not `textposition - widths/2` will be re-centered on the
#     textposition.

export TextRepelAlgorithm

"""
    TextRepelAlgorithm(; force, force_point, force_pull, only_move,
                         box_padding, point_padding)

Algorithm object for `Makie.annotation!`'s `algorithm` attribute. Wraps the
deterministic force-directed solver used by `textrepel!`.
"""
Base.@kwdef struct TextRepelAlgorithm
    force::NTuple{2,Float64}       = (1.0, 1.0)
    force_point::NTuple{2,Float64} = (1.0, 1.0)
    force_pull::NTuple{2,Float64}  = (0.01, 0.01)
    only_move::Symbol              = :both
    box_padding::Float64           = 4.0
    point_padding::Float64         = 0.0
end

function Makie.calculate_best_offsets!(
        alg::TextRepelAlgorithm,
        offsets::Vector{<:Vec2},
        textpositions::Vector{<:Point2},
        textpositions_offset::Vector{<:Point2},
        text_bbs::Vector{<:Rect2},
        bbox::Rect2;
        maxiter::Union{Makie.Automatic, Int},
        labelspace::Symbol,
        reset::Bool,
    )
    n = length(offsets)
    n == 0 && return

    T = eltype(offsets)
    if reset
        fill!(offsets, zero(T))
    end

    # Honor manual offsets exactly like the default Automatic dispatch does.
    # Loop avoids StaticArrays broadcasting a Vec into each Vector element.
    if all(p -> all(isfinite, p), textpositions_offset)
        for i in 1:n
            d = textpositions_offset[i] - textpositions[i]
            offsets[i] = T(d[1], d[2])
        end
        return
    end

    anchors = [Point2f(p[1], p[2]) for p in textpositions]
    sizes   = [Vec2f(bb.widths[1], bb.widths[2]) for bb in text_bbs]
    bnds    = Rect2f(Float32(bbox.origin[1]),  Float32(bbox.origin[2]),
                     Float32(bbox.widths[1]),  Float32(bbox.widths[2]))

    mi = maxiter === Makie.automatic ? 200 : Int(maxiter)
    params = MakieTextRepel.RepelParams(;
        force         = alg.force,
        force_point   = alg.force_point,
        force_pull    = alg.force_pull,
        only_move     = alg.only_move,
        box_padding   = alg.box_padding,
        point_padding = alg.point_padding,
        max_iter      = mi,
        max_overlaps  = Inf,
        bounds        = bnds,
    )

    new_offsets, _ = MakieTextRepel.solve_repel(anchors, sizes, params)

    for i in 1:n
        offsets[i] = T(new_offsets[i][1], new_offsets[i][2])
    end
    return
end
