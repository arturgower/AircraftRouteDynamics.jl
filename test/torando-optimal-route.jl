include("../src/AircraftRouteDynamics.jl")

tornado_wind(x, y, wind_vel = 10.0) = wind_vel .* [-y+1.5, x-1.5] ./ (sqrt(x^2 + y^2))

wind_speed(x, y) = tornado_wind(x, y)
# data = [wind_speed_field(θθs[i], ϕϕs[i]) for i in CartesianIndices(θθs)]
# wind_data = WindData(θs, ϕs, data)

# Calculate an optimal route

    N = 180;
    dt = 0.016;
    altitude = 8.0;
    θφ_initial = [1.0,1.0];
    θφ_end = [2.0,2.0];
    initial_velocity = [10.0,-1.0,0.0];
    
    norm(initial_velocity)
    # initial_velocity = [0.00006,0.0,0.0];

    aircraft = Aircraft(
        altitude = altitude,
        empty_weight = 10.0,
        drag_coefficient = 12.0,
        fuel_burn_rate = 10.0,
        fuel = 50.0,
        typical_speed = 8.0
    )

    initial_velocity = [10.0,0.0,0.0];
    setup = RouteSetup(
        aircraft = aircraft,
        iterations = N, dt = dt,
        θφ_initial = θφ_initial,
        θφ_end = θφ_end,
        initial_velocity = initial_velocity,
        tol = 1e-2,
    )

    function f(x, setup, wind_speed)
        turns, fuel = x_to_control_variables(x,setup)
        r = route(setup, fuel, turns, wind_speed)

        return objective(r, setup)
        # return objective_fuel(r, setup)
    end

    options = Optim.Options(
        f_tol = 1e-8,
        iterations = 2660,
        # iterations = 160,
        # store_trace = true
    )

    numberofvariables = 18;
    i = Int(round(numberofvariables / 2))
    initial_guess = [zeros(i); ones(i) .* aircraft.fuel_burn_rate]
    res = optimize(x -> f(x,setup,wind_speed), initial_guess, options)
    # res = optimize(x -> f(x,setup), initial_guess, LBFGS(), options)
    res.minimum

    turns, fuel = x_to_control_variables(res.minimizer, setup) 

    # turns = 10 .* ones(N)
    r = route(setup, fuel, turns, wind_speed)
    r = clip_route(r, setup)

    # @reset setup.initial_velocity = [10.0,-1.0,0.0];
    
    function f(x, setup, wind_speed)
        turns, fuel = x_to_control_variables(x,setup)
        r = route(setup, fuel, turns, wind_speed)

        return objective_fuel(r, setup)
    end

    res = optimize(x -> f(x,setup,wind_speed), initial_guess, options)
    turns, fuel = x_to_control_variables(res.minimizer, setup) 

    r_f = route(setup, fuel, turns, wind_speed)
    r_f = clip_route(r_f, setup)

    # Plot the results
    using Plots

    gr(size = (440,360), markersize = 2.0, linewidth = 1.0)
    # wind_speed(x, y) = 0.01 .* wind_speed_interpol_2D(x, y, wind_data)
    wind_speed_plot(x, y) = 0.035 .* wind_speed(x, y)

    plot(setup,wind_speed_plot)

    plot!(r.θs,r.φs, label = "speed route", linewidth = 2.0)
    plot!(r_f.θs,r_f.φs, label = "fuel route", linewidth = 2.0)
    scatter!(setup.θφ_initial[1:1], setup.θφ_initial[2:2], label = "Start destination", markersize = 5.0)
    scatter!(setup.θφ_end[1:1], setup.θφ_end[2:2], label = "End destination", xlims = (0.95,2.1), ylims = (0.92,2.1), markersize = 5.0)
    plot!(legend = :topleft, xlab = "latitude", ylab = "longitude")
    # savefig("tornado-compare.pdf")  


anim = @animate for i in 1:length(r.θs)

    wind_plot(setup,wind_speed_plot)
    
    if i < length(r.θs)
        plot!(r.θs[1:i],r.φs[1:i], label = "speed route", linewidth = 3.0)
    else    
        plot!(r.θs,r.φs, label = "speed route", linewidth = 3.0)
    end    
    if i < length(r_f.θs)
        plot!(r_f.θs[1:i],r_f.φs[1:i], 
        label = "fuel route", linewidth = 3.0)
    else    
        plot!(r_f.θs,r_f.φs, 
        label = "fuel route", linewidth = 3.0)
    end    
    scatter!(setup.θφ_initial[1:1], setup.θφ_initial[2:2], label = "Start", markersize = 5.0)
    scatter!(setup.θφ_end[1:1], setup.θφ_end[2:2], label = "End", xlims = (0.95,2.1), ylims = (0.92,2.1), markersize = 5.0, markercolor = :red)
    plot!(legend = :topleft, xlab = "latitude", ylab = "longitude")
end
# gif(anim,"tornado-compare.gif", fps = 8)

plot(r.fuel[1:127], linewidth = 2.0, lab = "speed route", ylab = "fuel", xlab = "time")
plot!(r_f.fuel, linewidth = 2.0, lab = "fuel route", ylab = "fuel", xlab = "time")
# savefig("tornado-fuel-compare.pdf")