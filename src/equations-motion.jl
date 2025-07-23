using LinearAlgebra, Statistics
using Optim
using Accessors

include("types.jl")
# include("functions_for_wind.jl")


# Definitions
# v_vec = velocity vector of plane
# v_vec = sv .* basis_align, where sv = norm(v)
# note v_vec is numerically written in terms of eθ and eφ, so that v_vec[1] is the component in the eθ direction.
# vp = cross(er,basis_align) and  basis_align = cross(vp,er)

# dmdt = mass rate in time
thrust_force(dmdt) = dmdt; 
function perpendicular_force(wind_vec, speed_old, basis_align, α) 
    return α * (speed_old -  dot(wind_vec, basis_align))^2 
end

function drag_force_2D(wind_vec, v_vec, aircraft::Aircraft)

    return -(aircraft.drag_coefficient * sqrt(sum((v_vec .- wind_vec).^2))) .* (v_vec - wind_vec)
end

function drag_force_3D(wind_vec, v_vec, aircraft::Aircraft)
    # U = wind_function(θ, ϕ, z)
    return -(aircraft.drag_coefficient * sqrt(sum((v_vec .- wind_vec).^2))) .* (v_vec - wind_vec)
end


function speed_new(speed, m, dmdt, dt, w_basis_align)
    dsdt = (w_basis_align - speed * dmdt - thrust_force(dmdt)) / m

    return speed + dsdt * dt;
end

function basis_perp_new(basis_perp_old, basis_align_old, wind_vec, speed_old, turn, mass, dt, external_force_perp)
   

    a = if speed_old ≈ 0.0 
        0.0
    else
        -(perpendicular_force(wind_vec, speed_old, basis_align_old, turn) + external_force_perp) / (mass * speed_old)
    end

    basis_perp = basis_perp_old + a*dt*basis_align_old  
    basis_perp = basis_perp ./ norm(basis_perp)

    return basis_perp
end

function θφ_new(θφ_old, v_vec, dt, r)
    
    dθdt = v_vec[1] / (r * sin(θφ_old[2]));   
    dφdt = v_vec[2] / r;

    θ_new = θφ_old[1] + dθdt * dt
    φ_new = θφ_old[2] + dφdt * dt

    if φ_new < 0
        φ_new = - φ_new
        φ_new = - φ_new
        θ_new = θ_new + pi
    elseif φ_new > pi
        φ_new =  φ_new - (φ_new - pi)
        θ_new = θ_new + pi
    end    

    if θ_new < 0
        θ_new = θ_new + 2pi
    elseif θ_new > 2pi 
        θ_new = θ_new - 2pi
    end  

    return [θ_new, φ_new]
end

# const empty_wind = WindData([-10.0, 10.0],[-10.0, 10.0], [[[0.0,0.0]] [[0.0,0.0]]; [[0.0,0.0]] [[0.0,0.0]]]);

function route(setup::RouteSetup, fuels, turns, wind_speed = (θ,φ) -> [0.0,0.0])
    aircraft = setup.aircraft

    # initialise data
    θs = zeros(N); φs = zeros(N);
    forces = [zeros(2) for i = 1:N];
    speeds = zeros(N);

    θs[1] = setup.θφ_start[1]
    φs[1] = setup.θφ_start[2]

    er = [0,0,1.0];
    r = aircraft.height; # height of flight

    v_vec = setup.v_vec_start;
    
    speeds[1] = norm(v_vec);

    if speeds[1] ≈ 0.0
        basis_align_old = [1.0,0.0,0.0]
    else
        basis_align_old = v_vec ./ norm(v_vec);
    end

    basis_perp_old = cross(er, basis_align_old)

    dt = setup.dt;
    flight_time = dt * setup.iterations;

    for i = 1:(N-1)
        dmdt = (fuels[i+1] - fuels[i]) / dt

        # external forces
        U = wind_speed(θs[i], φs[i]);
        U = [U; 0.0]
        drag = drag_force_2D(U, speeds[i] * basis_align_old, aircraft)
        external_force_align = dot(drag, basis_align_old);
        external_force_perp = dot(drag, basis_perp_old);

        forces[i] = drag;
        
        # update speed
        mass = fuels[i] + aircraft.empty_weight;
        speed = speed_new(speeds[i], mass, dmdt,dt,external_force_align)

        # update local coordinates
        basis_perp = basis_perp_new(basis_perp_old, basis_align_old, U, speeds[i], turns[i], mass, dt, external_force_perp)
        basis_align = cross(basis_perp,er)

        θφ = θφ_new([θs[i],φs[i]], speed .* basis_align, dt, aircraft.height)

        θs[i+1] = θφ[1];
        φs[i+1] = θφ[2];
        speeds[i+1] = speed

        if norm(θφ - setup.θφ_end) < setup.tol || fuels[i] < 0
            θs = θs[1:1+i]
            φs = φs[1:1+i]
            turns = turns[1:i+1]
            speeds = speeds[1:i+1]
            forces = forces[1:i+1]
            fuels = fuels[1:i+1]
            flight_time = dt * (i+1)

            break
        end

        basis_align_old = basis_align;
        basis_perp_old = basis_perp;
    end

    return Route(flight_time = flight_time, turns = turns, fuels = fuels, θs = θs, φs = φs, forces = forces, speeds = speeds)
end

function clip_route(r::Route, setup::RouteSetup)

    θ2 = setup.θφ_end[1]; φ2 = setup.θφ_end[2];
    i = findmin(abs.(r.θs .- θ2).^2 + abs.(r.φs .- φ2).^2)[2]

    @reset r.turns = r.turns[1:i]
    @reset r.speeds = r.speeds[1:i]
    @reset r.θs = r.θs[1:i]
    @reset r.φs = r.φs[1:i]
    @reset r.flight_time = setup.dt * i

    return r
end

function objective(r::Route, setup::RouteSetup)

    # turning_penalty = mean(exp.(route.turns.^4) .- 1.0)
    # turning_penalty = 0.0

    aircraft = setup.aircraft

    r = clip_route(r, setup);

    θ1 = setup.θφ_start[1]; φ1 = setup.θφ_start[2];
    θ2 = setup.θφ_end[1]; φ2 = setup.θφ_end[2];

    δ = 1e-7
    regulariser = δ * mean(abs.(r.turns)) 

    # regulariser = regulariser + δ * mean(abs.(r.speeds .- aircraft.target_speed))
    # regulariser = regulariser + fuel_burn_rate...?

    obj = regulariser

    # Aim to reach final destination
    dist = abs(r.θs[end] - θ2)^2 + abs(r.φs[end] - φ2)^2
    obj = obj + dist

    # Aim to minimize flight time.
    # distance along great circle
    D = aircraft.height * acos(sin(φ1)*sin(φ2) + cos(φ1)*cos(φ2)*cos(θ1 - θ2))
    T = D / aircraft.typical_speed

    obj = obj + 1e-2 * r.flight_time / T

    # Aim to minimize fuel use
    # obj = obj - 1e-1 * r.fuel[end]

    return obj
end

function objective_fuel(r::Route, setup::RouteSetup)

    # turning_penalty = mean(exp.(route.turns.^4) .- 1.0)
    # turning_penalty = 0.0

    aircraft = setup.aircraft

    r = clip_route(r, setup);

    θ1 = setup.θφ_start[1]; φ1 = setup.θφ_start[2];
    θ2 = setup.θφ_end[1]; φ2 = setup.θφ_end[2];

    δ = 1e-5
    regulariser = δ * mean(abs.(r.turns)) 

    # regulariser = regulariser + δ * mean(abs.(r.speeds .- aircraft.target_speed))
    # regulariser = regulariser + fuel_burn_rate...?

    obj = regulariser

    # Aim to reach final destination
    dist = abs(r.θs[end] - θ2)^2 + abs(r.φs[end] - φ2)^2
    obj = obj + dist

    # Aim use of fuel
    
    obj = obj - 1e-2 * r.fuels[end] / aircraft.empty_weight;

    # Aim to minimize fuel use
    # obj = obj - 1e-1 * r.fuel[end]

    return obj
end

function stretch_vector(x, N)
    j = Int(ceil(N / length(x)))
    return repeat(x, inner = j)[1:N]
end

function x_to_control_variables(x, setup::RouteSetup)

    i = Int(round(length(x) / 2))
    turns = stretch_vector(x[1:i], setup.iterations)
    burn_rates = abs.(stretch_vector(x[i+1:end], setup.iterations))

    max_burn_rate = setup.aircraft.fuel_burn_rate * 1.4;
    min_burn_rate = setup.aircraft.fuel_burn_rate * 0.4;

    is = findall(burn_rates .> max_burn_rate)
    burn_rates[is] .= max_burn_rate

    is = findall(burn_rates .< min_burn_rate)
    burn_rates[is] .= min_burn_rate

    fuels = setup.aircraft.fuel .- cumsum(burn_rates) * setup.dt
    
    return turns, fuels
end