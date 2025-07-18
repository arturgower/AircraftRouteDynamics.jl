# All types and structs used in the code

struct Route
    flight_time::Float64
    # The turns over time
    turns::Vector{Float64}
    # The vector of fuel use over time
    fuels::Vector{Float64}
    # latitude and longitude over time. Position φ = 0 is currently the norht pole
    θs::Vector{Float64}
    φs::Vector{Float64}
    # ground speeds over time
    speeds::Vector{Float64}
    # drag forces during flight in θ, φ basis
    forces::Vector{Vector{Float64}}
end

function Route(;
    flight_time = 0.0,
    turns = [0.0],
    fuels = [0.0],
    θs = [0.0],
    φs = [0.0],
    speeds = [0.0],
    forces  = [[0.0,0.0]])
    
    return Route(flight_time, turns, fuels, θs, φs, speeds, forces)
end

struct Aircraft
    height::Float64
    drag_coefficient::Float64
    empty_weight::Float64
    fuel_burn_rate::Float64 # burn rate in time
    fuel::Float64 # total fuel
    typical_speed::Float64 # a typical speed to help non-dimensionalise
end

struct RouteSetup
    aircraft::Aircraft
    iterations::Int
    dt::Float64
    θφ_start::Vector{Float64}
    θφ_end::Vector{Float64}
    v_vec_start::Vector{Float64}
    tol::Float64
end

function Aircraft(;
        height = 8,
        drag_coefficient = 0.2,
        empty_weight = 2.0,
        typical_speed = 1.0,
        fuel_burn_rate = 6.0,
        fuel = 6.0
    )
    return Aircraft(height,drag_coefficient,empty_weight,fuel_burn_rate,fuel,typical_speed)
end

function RouteSetup(; 
        iterations = 100, 
        dt = 0.02, tol = 1e-1, 
        θφ_start = [0.0,0.0],
        θφ_end = [0.2,0.2],
        v_vec_start = [1.0,1.0],
        aircraft = Aircraft()
    )

    RouteSetup(aircraft,iterations,dt,θφ_start,θφ_end,v_vec_start,tol)
end