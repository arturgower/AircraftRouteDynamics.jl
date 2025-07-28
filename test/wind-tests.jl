# basic simulation tests

@testset "Orthogonal wind and const acceleration" begin

# If the wind is orthogonal to the movement, and there is no turning, then we can analytically integrate the equations of the motion to compare the result.

# This is not true for the current model. Would need to allow the direction of travel to be different than the direction the plane points. Then the direction the plane points would only change when turning, and not due to wind.

    N = 1000;
    dt = 0.001;
    height = 8.0;
    θφ_start = [1.0,1.0];
    v_vec_start = [1.0,0.0,0.0];

    aircraft = Aircraft(
        height = height,
        empty_weight = 4.0,
        drag_coefficient = 0.5
    )

    setup = RouteSetup(
        aircraft = aircraft,
        iterations = N, dt = dt, 
        θφ_start = θφ_start, 
        v_vec_start = v_vec_start,
        tol = 1e-2,
    )
    
    # fuel use over time
    fuels = LinRange(10,4,N);

    # turns over time
    turns = 0.0 .* fuels

    # wind speed vector
    wind_v = [0.0,0.0]
    wind_speed(x,y) = wind_v

    # calculate route
    r = route(setup, fuels, turns, wind_speed)

    # The solution can be explicilty integrated if the aircraft does not change direciton.
    vs = LinRange(r.speeds[1], r.speeds[end],1000);
    dv = vs[2] - vs[1];

    dmdt = (r.fuels[2] - r.fuels[1]) / setup.dt;
    CD = aircraft.drag_coefficient;
    CT = aircraft.fuel_efficiency_coefficient
    
    integrate_vs = dv ./ (CD .* vs .^ 2 .+ dmdt .* vs .+ dmdt * CT);
    integrate_vs[1] = integrate_vs[1] / 2
    integrate_vs[end] = integrate_vs[end] / 2 

    integrate_v = sum(integrate_vs)

    ts = LinRange(0, setup.iterations * setup.dt, 1000);
    dt = ts[2] - ts[1]

    integrate_ms= - dt ./ (dmdt .* ts .+ fuels[1] .+ aircraft.empty_weight);
    integrate_ms[1] = integrate_ms[end] / 2
    integrate_ms[end] = integrate_ms[end] / 2

    integrate_m = sum(integrate_ms)

    # simulation makes an error of around dt
    @test abs(integrate_m - integrate_v) / dt < 1
    

    # Test with wind 

        
    # fuel use over time
    fuels = LinRange(10,4,N);

    # turns over time
    turns = 0.0 .* fuels .- 0.3

    # wind speed vector
    wind_v = [0.5,0.3]
    wind_speed(x,y) = wind_v

    # calculate route
    r = route(setup, fuels, turns, wind_speed)

    @test true 
end
    # using Plots
    # scatter(r.θs,r.φs, label = "route") 
