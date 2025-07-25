include("../../src/AircraftRouteDynamics.jl")

# Calculate an optimal route

    N = 45;
    dt = 0.02;
    height = 8.0;
    θφ_end = [1.5,1.8];
    θφ_start = [1.0,1.0];

    # initial velocity in θ and φ direction
    v_vec_start = [0.2,0.0,0.0];
    # v_vec_start = [0.00006,0.0,0.0];

    aircraft = Aircraft(
        height = height,
        empty_weight = 4.0,
        drag_coefficient = 0.5,
    )

    setup = RouteSetup(
        aircraft = aircraft,
        iterations = N, dt = dt, 
        θφ_start = θφ_start, 
        θφ_end = θφ_end,
        v_vec_start = v_vec_start,
        tol = 1e-2,
    )
    

     # fuel use over time
    fuels = LinRange(10,4,N);

    # wind speed vector in θ and φ direction
    wind_v = [1.0,1.0]
    wind_speed(x,y) = wind_v

    wind_speed_plot(x, y) = 0.002 .* wind_speed(x,y)

    using Plots

    plot(setup, wind_speed_plot; 
        θmin = 1.0, θmax = 1.08,
        φmin = 0.993, φmax = 1.007
    )

    map(0:2) do i 
        # turns over time
        turns = 0.0 .* fuels .- i * 2

        # calculate route
        r = route(setup, fuels, turns, wind_speed)
        scatter!(r.θs,r.φs, label = "route turn - $(i)") 
    end
    scatter!(xlab = "θ", ylab = "φ") 
    savefig("turn-wind-example.pdf")

    N = 65
     # fuel use over time
    fuels = LinRange(10,4,N);

    aircraft = Aircraft(
        height = height,
        empty_weight = 4.0,
        drag_coefficient = 3.3,
    )

    setup = RouteSetup(
        aircraft = aircraft,
        iterations = N, dt = dt, 
        θφ_start = θφ_start, 
        θφ_end = θφ_end,
        v_vec_start = v_vec_start,
        tol = 1e-2,
    )


    plot()
    map(0:2) do i 
        # turns over time
        turns = 0.0 .* fuels .- i * 2

        # calculate route
        r = route(setup, fuels, turns, wind_speed)
        scatter!(r.θs,r.φs, label = "route turn - $(i)") 
    end
    scatter!(xlab = "θ", ylab = "φ") 

        # wind speed vector in θ and φ direction
    wind_v = [-1.0,1.0]
    wind_speed(x,y) = wind_v

    wind_speed_plot(x, y) = 0.002 .* wind_speed(x,y)

    plot!(setup,wind_speed_plot; 
        θmin = 0.99, θmax = 1.025,
        φmin = 0.99, φmax = 1.065
    )

    savefig("wind-dominant-example.pdf")