using RigidBodyDynamics
using LinearAlgebra
using StaticArrays
using MeshCat
using MeshCatMechanisms
using MechanismGeometries
using CoordinateTransformations
using GeometryBasics
using Revise
using PitchPrediction
println("Libraries imported.")

src_dir = dirname(pathof(PitchPrediction))
urdf_file = joinpath(src_dir, "..", "urdf", "toy_vehicle.urdf")
ctlr_file = joinpath(src_dir, "toy-problem", "PDCtlr.jl")
traj_file = joinpath(src_dir, "toy-problem", "TrajGen.jl")

# include(traj_file)
# using .PDCtlr
# revise(PDCtlr)

vis = Visualizer()
mechanism_toy = parse_urdf(urdf_file; gravity = [0.0, 0.0, -9.81])

delete!(vis)
visuals = URDFVisuals(urdf_file)
mvis_toy = MechanismVisualizer(mechanism_toy, URDFVisuals(urdf_file), vis[:toy])
render(mvis_toy)

#%%
include(ctlr_file)

# Initialize the mechanism state
state = MechanismState(mechanism_toy)

# Set initial position 
free_joint, joint1, joint2 = joints(mechanism_toy)
zero!(state)
set_configuration!(state, joint1, 0)

# Set up the controller 
Δt = 1e-3
PDCtlr.ctlr_setup(mechanism_toy, state; time_step=Δt)

ts, qs, vs = simulate(state, 3.0, PDCtlr.pd_control!; Δt);

MeshCatMechanisms.animate(mvis_toy, ts, qs; realtimerate = 1.);

#%%