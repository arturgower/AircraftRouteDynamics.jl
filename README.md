# AircraftRouteDynamics

[![CI][ci-img]][ci-url] 

[ci-img]: https://github.com/arturgower/AircraftRouteDynamics.jl/actions/workflows/ci.yml/badge.svg
[ci-url]: https://github.com/arturgower/AircraftRouteDynamics.jl/actions/workflows/ci.yml

*A Julia library for simulating and calculating optimal routes for commercial aircrafts. 

The package uses some simplified dynamics to capture the main contributions to drag, wind, and thrust. Routes can be chosen to minimise fuel or flight time.

## Installation
This package is currently not registered, so to install it you will need to type in the Julia terminal: 
```julia
julia> ]
pkg> add https://github.com/arturgower/AircraftRouteDynamics.jl
then press the backspace key followed by
```julia
julia> using AircraftRouteDynamics
```

## Route simulation

Here we give a simple example 