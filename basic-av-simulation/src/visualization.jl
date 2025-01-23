"""
Plots trajectories and halfspaces given. Uses Δ as timestep for simulation.
"""
function plot_trajectories(trajectories::Vector{Trajectory}, halfspaces::Vector{HalfSpace}; Δ=0.25, fig_len=15.0)
    f = Figure()
    ax = f[1, 1] = Axis(f, aspect=DataAspect())
    xlims!(ax, -fig_len, fig_len)
    ylims!(ax, -fig_len, fig_len)

    for hs in halfspaces
        if abs(hs.a[2]) > 1e-3 # only plot if line isn't vertical. 
            xcoords = [-fig_len, fig_len]
            ycoords = (hs.b .- xcoords .* hs.a[1]) ./ hs.a[2]
            lines!(xcoords, ycoords, color=:black, linewidth=3)
        end
    end

    ps = [Observable(Point2f(t.x₀[1], t.x₀[2])) for t in trajectories]
    θs = [Observable(t.x₀[4]) for t in trajectories]
    Rs = [@lift([cos($θ) -sin($θ); sin($θ) cos($θ)]) for θ in θs]
    all_corners = [@lift(Point2f[$p.+$R*[t.vehicle_properties.half_length; t.vehicle_properties.half_width],
        $p.+$R*[t.vehicle_properties.half_length; -t.vehicle_properties.half_width],
        $p.+$R*[-t.vehicle_properties.half_length; -t.vehicle_properties.half_width],
        $p.+$R*[-t.vehicle_properties.half_length; t.vehicle_properties.half_width]]) for (p, R, t) in zip(ps, Rs, trajectories)]
    colors = [:blue; [:red for _ in 1:length(trajectories)-1]]

    for (corners, color) in zip(all_corners, colors)
        poly!(corners, color=color, strokecolor=:black, strokewidth=3)
    end
    display(f)
    sleep(Δ)
    states = [t.X for t in trajectories]
    for xs in zip(states...)
        for (e, x) in enumerate(xs)
            ps[e][] = Point2f(x[1], x[2])
            θs[e][] = x[4]
        end
        display(f)
        sleep(Δ)
    end
end
