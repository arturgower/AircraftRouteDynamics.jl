include("equations-motion.jl")

# Calculate an optimal route

    N = 50;
    dt = 0.02;
    height = 8.0;
    θφ_end = [1.5,1.8];
    θφ_start = [1.0,1.0];
    v_vec_start = [0.0,0.0,0.0];
    # v_vec_start = [0.00006,0.0,0.0];

    aircraft = Aircraft(
        height = height,
        empty_weight = 4.0,
        drag_coefficient = 0.5,
        fuel_burn_rate = 6.0
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
    fuel = LinRange(10,4,N);

    # fuel over time
    turns = 3.0 .* cos.(fuel)

    wind_v = [1.0,1.0] .* 10.0
    wind_speed(x,y) = wind_v

    # calculate route
    fuel = fuel .* 0;
    turns = turns .* 0;
    r = route(setup, fuel, turns, wind_speed)

    # Plot the results 
    using Plots
    include("plot.jl")

    wind_speed_plot(x, y) = 0.006 .* wind_speed(x,y)

    wind_plot(setup,wind_speed_plot)
    scatter!(r.θs,r.φs, label = "route")
    scatter!(setup.θφ_start[1:1], setup.θφ_start[2:2], label = "Start destination")



    # Generate a smooth random turn

    # moving_average(vs,n) = [sum(@view vs[i:(i+n-1)])/n for i in 1:(length(vs)-(n-1))]

    # turns = rand(20N) .- 0.5;
    # turns = moving_average(turns, 19N+1);
    # turns = 3.0 .* turns .* (1.4 / mean(abs.(turns)))

    # plot(turns)

    # # calculate route
    # r = route(setup, fuel, turns)
    # scatter(r.θs,r.φs, label = "route") 
