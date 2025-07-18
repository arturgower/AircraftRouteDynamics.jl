include("equations-motion.jl")

tornado_wind(x, y, wind_vel = 10.0) = wind_vel .* [-y+1.5, x-1.5] ./ (sqrt(x^2 + y^2))

wind_speed(x, y) = tornado_wind(x, y)
# data = [wind_speed_field(θθs[i], ϕϕs[i]) for i in CartesianIndices(θθs)]
# wind_data = WindData(θs, ϕs, data)

# Calculate an optimal route

    N = 100;
    dt = 0.03;
    height = 8.0;
    θφ_start = [1.0,1.0];
    θφ_end = [2.0,2.0];
    v_vec_start = [3.6,3.0,0.0];
    v_vec_start = [3.6,-3.0,0.0];
    v_vec_start = [7.0,5.0,0.0];
    
    norm(v_vec_start)
    # v_vec_start = [0.00006,0.0,0.0];

    aircraft = Aircraft(
        height = height,
        empty_weight = 4.0,
        drag_coefficient = 2.0,
        fuel_burn_rate = 12.0,
        fuel = 10.0,
        typical_speed = 8.0
    )

    setup = RouteSetup(
        aircraft = aircraft,
        iterations = N, dt = dt,
        θφ_start = θφ_start,
        θφ_end = θφ_end,
        v_vec_start = v_vec_start,
        tol = 1e-2,
    )

    # fuel over time
    fuel = 2 .* LinRange(10,4,N);
    burn_rates = diff(fuel) ./ dt

    # fuel over time
    turns = 3.0 .* cos.(fuel) .* 0.0
    fuel = fuel .* 0.0
    r = route(setup, fuel, turns, wind_speed)

    # Plot the results
    using Plots
    include("plot.jl")

    # wind_speed(x, y) = 0.01 .* wind_speed_interpol_2D(x, y, wind_data)
    wind_speed_plot(x, y) = 0.01 .* wind_speed(x, y)

    wind_plot(setup,wind_speed_plot)

    scatter!(r.θs,r.φs, label = "route")
    scatter!(setup.θφ_start[1:1], setup.θφ_start[2:2], label = "Start destination")
