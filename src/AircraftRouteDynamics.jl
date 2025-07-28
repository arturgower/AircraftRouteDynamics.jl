module AircraftRouteDynamics

using LinearAlgebra, Statistics
using Optim
using Accessors
using RecipesBase

# the main types
export Aircraft, Route, RouteSetup
include("types.jl")

# main forces 
export thrust_force, perpendicular_force, drag_force_2D

# calculate new route 
export route, clip_route

# functions for optimisation
export objective, objective_fuel
export stretch_vector, x_to_control_variables

include("equations-motion.jl")

# provides convieniant plot functions, though requries package: Plots
include("../test/plot.jl")

end # module
