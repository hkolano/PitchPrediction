using RigidBodyDynamics
using LinearAlgebra, StaticArrays, DataStructures
using MeshCat, MeshCatMechanisms, MechanismGeometries
using CoordinateTransformations
using GeometryBasics
using Printf, Plots, CSV, Tables, ProgressBars, WAV
using PitchPrediction

# Path of the source directory
src_dir = dirname(pathof(PitchPrediction))
# URDF of the seabotix vehicle + alpha arm
urdf_file = joinpath(src_dir, "..", "urdf", "alpha_seabotix.urdf")
frame_setup_file = joinpath(src_dir, "full-sim", "data-generation", "FrameSetup.jl")
hydro_calc_file = joinpath(src_dir, "full-sim", "data-generation", "HydroCalcWNoise.jl")
sim_file = joinpath(src_dir, "full-sim", "data-generation", "SimWExt.jl")
ctlr_file = joinpath(src_dir, "full-sim", "data-generation", "PIDCtlrMain.jl")
traj_file = joinpath(src_dir, "full-sim", "TrajGenMain.jl")

println("Libraries and external files imported.")

#%% Re-import functions
include(frame_setup_file)
include(hydro_calc_file)
include(sim_file)
include(ctlr_file)
include(traj_file)

# ----------------------------------------------------------
#                 One-Time Mechanism Setup
# ----------------------------------------------------------

vis = Visualizer()
mech_sea_alpha = parse_urdf(urdf_file; floating=true, gravity = [0.0, 0.0, 0.0])
# mech_sea_alpha = parse_urdf(urdf_file; gravity = [0.0, 0.0, 0.0])

# delete!(vis)

# Create visuals of the URDFs
visuals = URDFVisuals(urdf_file)
mvis = MechanismVisualizer(mech_sea_alpha, URDFVisuals(urdf_file), vis[:alpha])
render(mvis)
#%%

# Name the joints and bodies of the mechanism
vehicle_joint, base_joint, shoulder_joint, elbow_joint, wrist_joint = joints(mech_sea_alpha)
~, vehicle_body, shoulder_body, upper_arm_body, elbow_body, wrist_body = bodies(mech_sea_alpha)

render(mvis)