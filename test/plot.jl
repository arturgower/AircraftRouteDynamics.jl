using RecipesBase

@recipe function plot(setup::RouteSetup, wind_speed::Function; 
        Nθ = 15, Nφ = 14,
        θmin = 0.98 * min(setup.θφ_start[1], setup.θφ_end[1]),
        φmin = 0.98 * min(setup.θφ_start[2], setup.θφ_end[2]),
        θmax = 1.02 * max(setup.θφ_start[1], setup.θφ_end[1]),
        φmax = 1.02 * max(setup.θφ_start[2], setup.θφ_end[2]),
    )

    dθ = (θmax - θmin) / Nθ;
    dφ = (φmax - φmin) / Nφ;

    θs = θmin:dθ:θmax |> collect
    φs = φmin:dφ:φmax |> collect

    θθs = [θ for θ in θs, ϕ in φs]
    ϕϕs = [ϕ for θ in θs, ϕ in φs]

    quiver := wind_speed

    seriestype --> :quiver

    θθs, ϕϕs
end

