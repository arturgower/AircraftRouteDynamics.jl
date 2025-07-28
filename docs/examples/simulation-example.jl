include("../../src/AircraftRouteDynamics.jl")

# Calculate an optimal route

    N = 45;
    dt = 0.02;
    altitude = 8.0;
    θφ_end = [1.5,1.8];
    θφ_initial = [1.0,1.0];

    # initial velocity in θ and φ direction
    initial_velocity = [0.2,0.0,0.0];
    # initial_velocity = [0.00006,0.0,0.0];

    aircraft = Aircraft(
        altitude = altitude,
        empty_weight = 4.0,
        drag_coefficient = 0.5,
    )

    setup = RouteSetup(
        aircraft = aircraft,
        iterations = N, dt = dt, 
        θφ_initial = θφ_initial, 
        θφ_end = θφ_end,
        initial_velocity = initial_velocity,
        tol = 1e-2,
    )
    

     # fuel use over time
    fuel = LinRange(10,4,N);

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
        turns = 0.0 .* fuel .- i * 2

        # calculate route
        r = route(setup, fuel, turns, wind_speed)
        scatter!(r.θs,r.φs, label = "route turn - $(i)") 
    end
    scatter!(xlab = "θ", ylab = "φ") 
    savefig("turn-wind-example.pdf")

    N = 65
     # fuel use over time
    fuel = LinRange(10,4,N);

    aircraft = Aircraft(
        altitude = altitude,
        empty_weight = 4.0,
        drag_coefficient = 3.3,
    )

    setup = RouteSetup(
        aircraft = aircraft,
        iterations = N, dt = dt, 
        θφ_initial = θφ_initial, 
        θφ_end = θφ_end,
        initial_velocity = initial_velocity,
        tol = 1e-2,
    )


    plot()
    map(0:2) do i 
        # turns over time
        turns = 0.0 .* fuel .- i * 2

        # calculate route
        r = route(setup, fuel, turns, wind_speed)
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