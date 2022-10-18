module PitchPrediction 

using RigidBodyDynamics
using LinearAlgebra, StaticArrays, DataStructures
using MeshCat, MeshCatMechanisms, MechanismGeometries
using CoordinateTransformations
using GeometryBasics
using Printf, Plots, CSV, Tables, ProgressBars, Revise
using JLD

include("full-sim/data-generation/FrameSetup.jl")
include("full-sim/data-generation/HydroCalc.jl")
include("full-sim/data-generation/SimWExt.jl")
include("full-sim/data-generation/PIDCtlr_vehicleonly.jl")
include("full-sim/data-generation/TrajGenMain.jl")

end # module
