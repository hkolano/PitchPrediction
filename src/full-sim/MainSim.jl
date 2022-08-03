# ----------------------------------------------------------
#                     Import Libraries
# ----------------------------------------------------------
#%%
using RigidBodyDynamics
using LinearAlgebra, StaticArrays
using MeshCat, MeshCatMechanisms, MechanismGeometries
using CoordinateTransformations
using GeometryBasics
using Printf
using PitchPrediction

# include("/home/hkolano/onr-dynamics-julia/simulate_with_ext_forces.jl")


# Path of the source directory
src_dir = dirname(pathof(PitchPrediction))
# URDF of the seabotix vehicle + alpha arm
urdf_file = joinpath(src_dir, "..", "urdf", "alpha_seabotix.urdf")
frame_setup_file = joinpath(src_dir, "full-sim", "FrameSetup.jl")
hydro_calc_file = joinpath(src_dir, "full-sim", "HydroCalc.jl")
sim_file = joinpath(src_dir, "full-sim", "SimWExt.jl")

println("Libraries and external files imported.")

#%% Re-import functions
include(frame_setup_file)
include(hydro_calc_file)
include(sim_file)

# ----------------------------------------------------------
#                 One-Time Mechanism Setup
# ----------------------------------------------------------
#%%
vis = Visualizer()
mech_sea_alpha = parse_urdf(urdf_file; floating=true, gravity = [0.0, 0.0, 0.0])
# mech_sea_alpha = parse_urdf(urdf_file; gravity = [0.0, 0.0, 0.0])

delete!(vis)

# Create visuals of the URDFs
visuals = URDFVisuals(urdf_file)
mvis = MechanismVisualizer(mech_sea_alpha, URDFVisuals(urdf_file), vis[:alpha])
render(mvis)

# Name the joints and bodies of the mechanism
vehicle_joint, base_joint, shoulder_joint, elbow_joint, wrist_joint = joints(mech_sea_alpha)
~, vehicle_body, shoulder_body, upper_arm_body, elbow_body, wrist_body = bodies(mech_sea_alpha)
num_virtual_links = 0

body_frame = default_frame(vehicle_body)
shoulder_frame = default_frame(shoulder_body)
upper_arm_frame = default_frame(upper_arm_body)
elbow_frame = default_frame(elbow_body)
wrist_frame = default_frame(wrist_body)
base_frame = root_frame(mech_sea_alpha)
# setelement!(mvis_alpha, shoulder_frame)

println("Mechanism built.")

# ----------------------------------------------------------
#                 Buoyancy Setup
# ----------------------------------------------------------
#%%
frame_names_cob = ["vehicle_cob", "shoulder_cob", "ua_cob", "elbow_cob", "wrist_cob"]
frame_names_com = ["vehicle_com", "shoulder_com", "ua_com", "elbow_com", "wrist_com"]
cob_vecs = [SVector{3, Float64}([0.0, 0.0, -.01]), SVector{3, Float64}([-0.001, -0.003, .032]), SVector{3, Float64}([0.073, 0.0, -0.002]), SVector{3, Float64}([0.015, -0.012, -.003]), SVector{3, Float64}([0.0, 0.0, -.098])]
com_vecs = [SVector{3, Float64}([0.0, 0.0, -0.06]), SVector{3, Float64}([0.005, -.001, 0.016]), SVector{3, Float64}([0.073, 0.0, 0.0]), SVector{3, Float64}([0.017, -0.026, -0.002]), SVector{3, Float64}([0.0, 0.003, -.098])]
cob_frames = []
com_frames = []
setup_frames!(mech_sea_alpha, frame_names_cob, frame_names_com, cob_vecs, com_vecs, cob_frames, com_frames)

# buoyancy force setup
# ------------------------------------------------------------------------
#                           BUOYANCY SETUP
# ------------------------------------------------------------------------
# f = rho * g * V
# f = 997 (kg/m^3) * 9.81 (m/s^2) * V_in_L *.001 (m^3) = kg m / s^2
# One time setup of buoyancy forces
rho = 997
volumes = [22.2/(.001*rho), .018, .203, .025, .155, .202] # vehicle, shoulder, ua, elbow, wrist, armbase
buoy_force_mags = volumes * 997 * 9.81 * .001
buoy_lin_forces = []
for mag in buoy_force_mags
    lin_force = FreeVector3D(base_frame, [0.0, 0.0, mag])
    push!(buoy_lin_forces, lin_force)
end
print(buoy_lin_forces)

masses = [22.2, .194, .429, .115, .333, .341]
grav_forces = masses*9.81
grav_lin_forces = []
for f_g in grav_forces
    lin_force = FreeVector3D(base_frame, [0.0, 0.0, -f_g])
    push!(grav_lin_forces, lin_force)
end

drag_link1 = [0.26 0.26 0.3]*rho
drag_link2 = [0.3 1.6 1.6]*rho
drag_link3 = [0.26 0.3 0.26]*rho
drag_link4 = [1.8 1.8 0.3]*rho
link_drag_coeffs = [drag_link1, drag_link2, drag_link3, drag_link4]

println("CoM and CoB frames initialized. \n")

# ----------------------------------------------------------
#                 State Initialization
# ----------------------------------------------------------
#%%
state = MechanismState(mech_sea_alpha)
zero!(state)
set_configuration!(state, vehicle_joint, [0., 0., 0.0, 0., 0., 0.])

# Define desired accelerations
des_acc = similar(velocity(state))
fill!(des_acc[vehicle_joint], 0.0)
des_acc[vehicle_joint][1] = 1.0
des_acc[base_joint][1] = 0.0
des_acc[shoulder_joint][1] = 0.0
des_acc[elbow_joint][1] = 0.0
des_acc[wrist_joint][1] = 0.0

# ----------------------------------------------------------
#                          Simulate
# ----------------------------------------------------------
final_time = 5.
ts, qs, vs = simulate_with_ext_forces(state, final_time, hydro_calc!, simple_control!; Δt = 5e-3)
# ts, qs, vs = simulate(state_alpha, final_time, simple_control!; Δt = 1e-2)

println("Simulation finished.")

#%% Animate
render(mvis_alpha)
MeshCatMechanisms.animate(mvis_alpha, ts, qs; realtimerate = 0.5)