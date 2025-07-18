include("equations-motion.jl")

# Calculate an optimal route
    N = 120;
    dt = 0.02;
    height = 8.0;
    θφ_start = [1.0,1.0];
    θφ_end = [1.5,1.8];
    v_vec_start = [1.5,1.0,0.0];

    aircraft = Aircraft(
        height = height,
        empty_weight = 6.0,
        drag_coefficient = 0.5,
        fuel_burn_rate = 6.0,
        fuel = 18.0,
        typical_speed = 2.0
    )

    setup = RouteSetup(
        aircraft = aircraft,
        iterations = N, dt = dt, 
        θφ_start = θφ_start, 
        θφ_end = θφ_end,
        v_vec_start = v_vec_start,
        tol = 1e-2,
    )

    wind_v = [1.0,1.0] .* 1.0
    wind_speed(x,y) = wind_v

    function f(x, setup, wind_speed)
        turns, fuels = x_to_control_variables(x,setup)
        r = route(setup, fuels, turns, wind_speed)

        return objective(r, setup)
    end

    options = Optim.Options(
        f_tol = 1e-8,
        iterations = 660,
        # iterations = 160,
        # store_trace = true
    )

    numberofvariables = 16;
    i = Int(round(numberofvariables / 2))
    initial_guess = [zeros(i); ones(i) .* aircraft.fuel_burn_rate]
    res = optimize(x -> f(x,setup,wind_speed), initial_guess, options)
    # res = optimize(x -> f(x,setup), initial_guess, LBFGS(), options)
    res.minimum

    turns, fuels = x_to_control_variables(res.minimizer, setup) 

    # turns = 10 .* ones(N)
    r = route(setup, fuels, turns, wind_speed)
    r = clip_route(r, setup)

    # r = route(setup, ms, turns .* 0.0)
    objective(r, setup)

    norm(setup.θφ_end - [r.θs[end], r.φs[end]])^2

    using Plots
    include("plot.jl")

    wind_speed_plot(x, y) = 0.03 .* wind_speed(x, y)

    wind_plot(setup,wind_speed_plot)

    scatter!(r.θs,r.φs, label = "route")
    scatter!(setup.θφ_end[1:1], setup.θφ_end[2:2], label = "Target destination", markersize = 3.0)
    # savefig("constant-wind-optimal.pdf")

    # plot(turns)
    # plot(diff(fuels) ./ dt) 
