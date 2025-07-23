function wind_plot(setup, wind_speed; Nθ = 15, Nϕ = 14)

    θmin = 0.95 * min(setup.θφ_start[1], setup.θφ_end[1]);
    θmax = 1.05 * max(setup.θφ_start[1], setup.θφ_end[1]);
    ϕmin = 0.95 * min(setup.θφ_start[2], setup.θφ_end[2]);
    ϕmax = 1.05 * max(setup.θφ_start[2], setup.θφ_end[2]);

    dθ = (θmax - θmin) / Nθ;
    dϕ = (ϕmax - ϕmin) / Nϕ;

    θs = θmin:dθ:θmax |> collect
    ϕs = ϕmin:dϕ:ϕmax |> collect

    θθs = [θ for θ in θs, ϕ in ϕs]
    ϕϕs = [ϕ for θ in θs, ϕ in ϕs]

    quiver(θθs, ϕϕs, quiver = wind_speed)
end

