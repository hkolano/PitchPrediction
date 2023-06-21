mutable struct jointState
    θs::Array{Float64}
    dθs::Array{Float64}
end

mutable struct Waypoints
    start::jointState 
    goal::jointState
end
