# All types and structs used in the code

struct Route
    flight_time::Float64
    # The turns over time
    turns::Vector{Float64}
    # The vector of fuel use over time
    fuel::Vector{Float64}
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
    fuel = [0.0],
    θs = [0.0],
    φs = [0.0],
    speeds = [0.0],
    forces  = [[0.0,0.0]])
    
    return Route(flight_time, turns, fuel, θs, φs, speeds, forces)
end

struct Aircraft
    altitude::Float64
    drag_coefficient::Float64
    fuel_efficiency_coefficient::Float64
    empty_weight::Float64
    fuel_burn_rate::Float64 # burn rate in time
    fuel::Float64 # total fuel
    typical_speed::Float64 # a typical speed to help non-dimensionalise
end

struct RouteSetup
    aircraft::Aircraft
    iterations::Int
    dt::Float64
    θφ_initial::Vector{Float64}
    θφ_end::Vector{Float64}
    initial_velocity::Vector{Float64}
    tol::Float64
end

function Aircraft(;
        fuel_efficiency_coefficient = 1.0,
        altitude = 8,
        drag_coefficient = 0.2,
        empty_weight = 2.0,
        typical_speed = 1.0,
        fuel_burn_rate = 6.0,
        fuel = 6.0
    )
    return Aircraft(altitude,drag_coefficient,fuel_efficiency_coefficient,empty_weight,fuel_burn_rate,fuel,typical_speed)
end

function RouteSetup(; 
        iterations = 100, 
        dt = 0.02, tol = 1e-1, 
        θφ_initial = [0.0,0.0],
        θφ_end = [0.2,0.2],
        initial_velocity = [1.0,1.0],
        aircraft = Aircraft()
    )

    RouteSetup(aircraft,iterations,dt,θφ_initial,θφ_end,initial_velocity,tol)
end