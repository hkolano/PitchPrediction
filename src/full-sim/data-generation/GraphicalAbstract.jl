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

end_eff_mass = 1.0
masses = [22.2, .194, .429, .115, .333, .341+end_eff_mass]
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

d_lin_coeffs = [4.5, 8.0, 2.7, 3.4, 4.6, 52.9]
d_nonlin_coeffs = [11.4, 20.0, 13.5, 34.4, 65.9, 132.3]

println("CoM and CoB frames initialized. \n")

# ----------------------------------------------------------
#                 State Initialization
# ----------------------------------------------------------
#%%

function reset_to_equilibrium!(state)
    zero!(state)
    set_configuration!(state, vehicle_joint, [.993607, 0., 0.11289, 0.00034, 0., 0., 0.])
end

# Constants
state = MechanismState(mech_sea_alpha)
Δt = 1e-3
final_time = 5.0
goal_freq = 50
sample_rate = Int(floor((1/Δt)/goal_freq))

render(mvis)

#%%
# (temporary adds while making changes to ctlr and traj generator)
include(ctlr_file)
# include(traj_file)

# ----------------------------------------------------------
#                      Gather Sim Data
# ----------------------------------------------------------

# num_trajs = 5000
#%%
# # Create (num_trajs) different trajectories and save to csvs
# for n in ProgressBar(1:num_trajs)
    include(traj_file)
    # Reset the sim to the equilibrium position
    reset_to_equilibrium!(state)
    # Start up the controller
    ctlr_cache = PIDCtlr.CtlrCache(Δt, mech_sea_alpha)

    # ----------------------------------------------------------
    #                          Simulate
    # ----------------------------------------------------------
    # Generate a random waypoint and see if there's a valid trajectory to it
    # include(traj_file)
    wp = TrajGen.gen_reaching_waypoints()
    traj = TrajGen.find_trajectory(wp)

    # Scale that trajectory to 1x-3x "top speed"
    scaled_traj = TrajGen.scale_trajectory(traj...)
    params = scaled_traj[1]
    duration = scaled_traj[2]
    poses = scaled_traj[3]
    vels = scaled_traj[4]

    # Make vector of waypoint values and time step to save to csv
    waypoints = [Δt*sample_rate params.wp.start.θs... params.wp.goal.θs... params.wp.start.dθs... params.wp.goal.dθs...]

    ts, qs, vs = simulate_with_ext_forces(state, duration, params, ctlr_cache, hydro_calc!, PIDCtlr.pid_control!; Δt=Δt)
    # ts2, qs2, vs2 = simulate_with_ext_forces(state, duration, params, ctlr_cache, hydro_calc!, PIDCtlr.pid_stationary!; Δt=Δt)
    # ts, qs, vs = simulate(state_alpha, final_time, simple_control!; Δt = 1e-2
    
    render(mvis)
    MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 1.0)
    # MeshCatMechanisms.animate(mvis, ts2, qs2; realtimerate = 3.0)
# end

println("Simulation finished.")

