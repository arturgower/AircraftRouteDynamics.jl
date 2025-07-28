import Pkg 
Pkg.add("Test")

using Test

include("../src/equations-motion.jl")
include("wind-tests.jl")