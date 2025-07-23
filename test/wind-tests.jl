# basic simulation tests

@testset "Orthogonal wind and const acceleration" begin

# If the wind is orthogonal to the movement, and there is no turning, then we can analytically integrate the equations of the motion to compare the result.

# This is not true for the current model. Would need to allow the direction of travel to be different than the direction the plane points. Then the direction the plane points would only change when turning, and not due to wind.

    N = 40;
    dt = 0.02;
    height = 8.0;
    θφ_start = [1.0,1.0];
    v_vec_start = [1.0,0.0,0.0];

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
        v_vec_start = v_vec_start,
        tol = 1e-2,
    )
    
    # fuel use over time
    fuel = LinRange(10,4,N);

    # turns over time
    turns = 0.0 .* fuel

    # wind speed vector
    wind_v = [0.0,0.0]
    wind_speed(x,y) = wind_v

    # calculate route
    r = route(setup, fuel, turns, wind_speed)


    drag = drag_force_2D([wind_v; 0.0], v_vec_start, aircraft)

    basis_align = v_vec_start / norm(v_vec_start);
    basis_perp = cross([0.0,0.0,1.0], basis_align)

    external_force_align = dot(drag, basis_align);
    external_force_perp = dot(drag, basis_perp);

    r.speeds[end] - r.speeds[1]

    @test true
end
    # Generate a smooth random turn

    # moving_average(vs,n) = [sum(@view vs[i:(i+n-1)])/n for i in 1:(length(vs)-(n-1))]

    # turns = rand(20N) .- 0.5;
    # turns = moving_average(turns, 19N+1);
    # turns = 3.0 .* turns .* (1.4 / mean(abs.(turns)))

    # plot(turns)

    # # calculate route
    # r = route(setup, fuel, turns)
    # scatter(r.θs,r.φs, label = "route") 
