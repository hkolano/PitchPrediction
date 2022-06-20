using RigidBodyDynamics
using LinearAlgebra
using StaticArrays
using MeshCat
using MeshCatMechanisms
using MechanismGeometries
using CoordinateTransformations
using GeometryBasics
using Revise
using Plots
using PitchPrediction
println("Libraries imported.")

src_dir = dirname(pathof(PitchPrediction))
urdf_file = joinpath(src_dir, "..", "urdf", "toy_vehicle.urdf")
ctlr_file = joinpath(src_dir, "toy-problem", "PDCtlr.jl")
traj_file = joinpath(src_dir, "toy-problem", "TrajGen.jl")
simulate_file = joinpath(src_dir, "toy-problem", "simulate_des_trajectory.jl")
# include(traj_file)
# using .PDCtlr
# revise(PDCtlr)

vis = Visualizer()
mechanism_toy = parse_urdf(urdf_file; gravity = [0.0, 0.0, -9.81])

delete!(vis)
visuals = URDFVisuals(urdf_file)
mvis_toy = MechanismVisualizer(mechanism_toy, URDFVisuals(urdf_file), vis[:toy])
render(mvis_toy)

#%% FUNCTIONS

function reset_to_equilibrium(state)
    set_configuration!(state, [0.18558, -0.18558, 0.0])
    set_velocity!(state, [0.0, 0.0, 0.0])
end

#%%
include(ctlr_file)
include(traj_file)
include(simulate_file)

# Initialize the mechanism state
state = MechanismState(mechanism_toy)

# Set initial position 
free_joint, joint1, joint2 = joints(mechanism_toy)
reset_to_equilibrium(state)

# Set up the controller 
Δt = 1e-3
ctlr_cache = PDCtlr.CtlrCache(Δt, mechanism_toy)

#%%
wp = TrajGen.gen_rand_waypoints_from_equil()
traj = TrajGen.find_trajectory(wp)

while traj === nothing
    global wp = TrajGen.gen_rand_waypoints_from_equil()
    global traj = TrajGen.find_trajectory(wp)
end

params = traj[1]
duration = traj[2]
println("Going to point $(wp.goal.θs)")

ts, qs, vs = simulate_des_trajectory(state, duration, params, ctlr_cache, PDCtlr.pd_control!; Δt);

MeshCatMechanisms.animate(mvis_toy, ts, qs; realtimerate = 1.);
#%%

# Plotting joint angles
qs1 = [qs[i][1] for i in 1:length(qs)]
qs2 = [qs[i][2] for i in 1:length(qs)]
qs3 = [qs[i][3] for i in 1:length(qs)]
plot(qs2)
#%%