# recipe.jl — the TextRepel Makie recipe.

@recipe TextRepel (positions,) begin
    "Labels to place: Vector of String / LaTeXString / rich text."
    text = nothing

    # ── Physics (pixel space, anisotropic) ──
    "Label↔label repulsion strength (x, y)."
    force = (1.0, 1.0)
    "Label↔data-point repulsion strength (x, y)."
    force_point = (1.0, 1.0)
    "Spring pull back to the anchor (x, y)."
    force_pull = (0.01, 0.01)
    "Maximum solver iterations."
    max_iter = 2000
    "Constrain movement: :both, :x, or :y."
    only_move = :both

    # ── Spacing / dropping ──
    "Pixels of padding around each label box for the solver."
    box_padding = 4.0
    "Pixel halo around each data point."
    point_padding = 0.0
    "Drop labels overlapping more than this many others (Inf = keep all)."
    max_overlaps = Inf

    # ── Connector segments ──
    "Draw connector lines from points to displaced labels."
    segments = true
    "Suppress a connector if the label moved fewer than this many pixels."
    min_segment_length = 5.0
    "Connector color."
    segmentcolor = :gray60
    "Connector width."
    linewidth = 1.0

    # ── Background box (geom_label_repel style) ──
    "Draw a filled background box behind each label."
    background = false
    "Background fill color."
    backgroundcolor = (:white, 0.8)
    "Background stroke color."
    strokecolor = :gray70
    "Background stroke width."
    strokewidth = 0.0
    "Background corner radius (px)."
    cornerradius = 4.0

    # ── Text passthrough ──
    fontsize = @inherit fontsize
    font = @inherit font
    color = @inherit textcolor
    align = (:center, :center)
end

# (xs, ys) → Vector{Point2f}
Makie.convert_arguments(::Type{<:TextRepel}, x::AbstractVector{<:Real}, y::AbstractVector{<:Real}) =
    (Point2f.(x, y),)

function Makie.plot!(p::TextRepel)
    # Minimal body; fully wired in Task 9.
    return p
end
