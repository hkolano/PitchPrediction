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

include("UVMSsetup.jl")
include("HydroCalc.jl")
include("SimWExt.jl")
include("PIDCtlr.jl")
include("TrajGenJoints.jl")
include("UVMSPlotting.jl")
include("HelperFuncs.jl")
include("ConfigFiles/MagicNumInvKin.jl")
include("ConfigFiles/MagicNumBlueROV.jl")
include("ConfigFiles/MagicNumAlpha.jl")

urdf_file = joinpath("urdf", "blue_rov_revjaw.urdf")

#%%
# ----------------------------------------------------------
#                 One-Time Mechanism Setup
# ----------------------------------------------------------
mech_blue_alpha, mvis, joint_dict, body_dict, frame_dict = mechanism_reference_setup(urdf_file)
cob_frame_dict, com_frame_dict = setup_frames(body_dict, body_names, cob_vec_dict, com_vec_dict)

println("Mechanism built.")