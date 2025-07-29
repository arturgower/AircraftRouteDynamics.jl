@recipe function plot(setup::RouteSetup, wind_speed::Function; 
        Nθ = 15, Nφ = 15,
        θmin = 0.98 * min(setup.θφ_initial[1], setup.θφ_end[1]),
        φmin = 0.98 * min(setup.θφ_initial[2], setup.θφ_end[2]),
        θmax = 1.02 * max(setup.θφ_initial[1], setup.θφ_end[1]),
        φmax = 1.02 * max(setup.θφ_initial[2], setup.θφ_end[2]),
        amplitude = 1.0
    )

    dθ = (θmax - θmin) / Nθ;
    dφ = (φmax - φmin) / Nφ;

    θs = θmin:dθ:θmax |> collect
    φs = φmin:dφ:φmax |> collect

    θθs = [θ for θ in θs, ϕ in φs]
    ϕϕs = [ϕ for θ in θs, ϕ in φs]

    wind_speed_plot(x, y) = amplitude .* wind_speed(x, y)

    quiver := wind_speed_plot

    seriestype --> :quiver

    θθs, ϕϕs
end

@recipe function plot(r::Route; step_size = 1)

    @series begin
        markersize --> 3
        label --> "Route"
        seriestype --> :scatter
        r.θs[1:step_size:end],r.φs[1:step_size:end]
    end

    markersize --> 4
    xlab --> "θ - longitude"
    ylab --> "φ - latitude"

    @series begin
        label := "Start"
        seriestype --> :scatter
        r.θs[1:1],r.φs[1:1]
    end
    
    @series begin
        label :="End"
        seriestype --> :scatter
        r.θs[end:end],r.φs[end:end]
    end
end

@recipe function plot(r::Route, wind_speed::Function; 
        Nθ = 15, Nφ = 15,
        θmin = 0.98 * minimum(r.θs),
        φmin = 0.98 * minimum(r.φs),
        θmax = 1.02 * maximum(r.θs),
        φmax = 1.02 * maximum(r.φs),
        wind_amplitude = 0.1
    )

    @series begin
        dθ = (θmax - θmin) / Nθ;
        dφ = (φmax - φmin) / Nφ;

        θs = θmin:dθ:θmax |> collect
        φs = φmin:dφ:φmax |> collect

        θθs = [θ for θ in θs, ϕ in φs]
        ϕϕs = [ϕ for θ in θs, ϕ in φs]

        wind_speed_plot(x, y) = wind_amplitude .* wind_speed(x, y)

        quiver := wind_speed_plot

        seriestype --> :quiver

        θθs, ϕϕs
    end

    @series begin
        r
    end
end

