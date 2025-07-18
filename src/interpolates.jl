using Interpolations 
using LinearAlgebra


tornado_wind(x, y, wind_vel = 10.3) = wind_vel .* [-y, x] ./ (sqrt(x^2 + y^2))


function interpolate_wind(θs, φs, vθs, vφs)

    itp_vθs=interpolate((θs, φs), vθs,Gridded(Linear()))
    itp_vφs=interpolate((θs, φs), vφs,Gridded(Linear()))

    function fwind(θ, φ)
        vθ=itp_vθs(θ, φ)
        vφ=itp_vφs(θ, φ)

        return [vθ, vφ]
    end
    return fwind # which is a function
end

θs=LinRange(-pi,pi,100)
φs=LinRange(0.01,pi,100)
vs=[tornado_wind(θ,φ) for θ in θs, φ in φs]

vθs=[vs[i][1] for i in CartesianIndices(vs)]
vφs=[vs[i][2] for i in CartesianIndices(vs)]

fwind=interpolate_wind(θs,φs,vθs,vφs)

fwind(θs[1], φs[1])

vs_inter=[fwind(θ,φ) for θ in θs, φ in φs]

vs_inter' |> collect

norm.(vs_inter-vs)
