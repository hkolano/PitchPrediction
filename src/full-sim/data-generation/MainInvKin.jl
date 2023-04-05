#= 
Main flight code for running the TPIK pipeline.
=# 

# ----------------------------------------------------------
#                     Import Libraries
# ----------------------------------------------------------
#%%
using RigidBodyDynamics, Rotations
using LinearAlgebra, StaticArrays, DataStructures
using MeshCat, MeshCatMechanisms, MechanismGeometries
using CoordinateTransformations
using GeometryBasics
using Printf, Plots, CSV, Tables, ProgressBars, Revise
using Random

include("HydroCalc.jl")
include("SimWExt.jl")
include("PIDCtlr.jl")
include("UVMSPlotting.jl")
include("HelperFuncs.jl")
include("Noiser.jl")

include("UVMSsetup.jl")
include("ConfigFiles/MagicNumInvKin.jl")
include("ConfigFiles/ConstMagicNums.jl")
include("ConfigFiles/MagicNumBlueROV.jl")
include("ConfigFiles/MagicNumAlpha.jl")

urdf_file = joinpath("urdf", "blue_rov_revjaw.urdf")

#%%
# ----------------------------------------------------------
#                 One-Time Mechanism Setup
# ----------------------------------------------------------
mech_blue_alpha, mvis, joint_dict, body_dict = mechanism_reference_setup(urdf_file)

include("TrajGenWorld.jl")
include("Tasks.jl")

cob_frame_dict, com_frame_dict = setup_frames(body_dict, body_names, cob_vec_dict, com_vec_dict)
buoyancy_force_dict, gravity_force_dict = setup_buoyancy_and_gravity(buoyancy_mag_dict, grav_mag_dict)

state = MechanismState(mech_blue_alpha)
num_dofs = num_velocities(mech_blue_alpha)
num_actuated_dofs = num_dofs-2
println("Mechanism built.")