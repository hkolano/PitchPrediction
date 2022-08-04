# ----------------------------------------------------------
#                     Import Libraries
# ----------------------------------------------------------
#%%
using RigidBodyDynamics
using LinearAlgebra, StaticArrays
using MeshCat, MeshCatMechanisms, MechanismGeometries
using CoordinateTransformations
using GeometryBasics
using Printf, Plots
using PitchPrediction

# include("/home/hkolano/onr-dynamics-julia/simulate_with_ext_forces.jl")


# Path of the source directory
src_dir = dirname(pathof(PitchPrediction))
# URDF of the seabotix vehicle + alpha arm
urdf_file = joinpath(src_dir, "..", "urdf", "alpha_seabotix.urdf")
frame_setup_file = joinpath(src_dir, "full-sim", "data-generation", "FrameSetup.jl")
hydro_calc_file = joinpath(src_dir, "full-sim", "data-generation", "HydroCalc.jl")
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
Δt = 1e-3
goal_freq = 50
sample_rate = Int(floor((1/Δt)/goal_freq))

zero!(state)
set_configuration!(state, vehicle_joint, [1., 0., 0., 0., 0., 0., 0.])

ctlr_cache = PIDCtlr.CtlrCache(Δt, mech_sea_alpha)


# ----------------------------------------------------------
#                          Simulate
# ----------------------------------------------------------
final_time = 2.5
ts, qs, vs = simulate_with_ext_forces(state, final_time, ctlr_cache, hydro_calc!, PIDCtlr.pid_control!; Δt=Δt)
# ts, qs, vs = simulate(state_alpha, final_time, simple_control!; Δt = 1e-2)

println("Simulation finished.")
paths = Dict();
ts_down = [ts[i] for i in 1:sample_rate:length(ts)]
for n = 8:11
    joint_poses = [qs[i][n] for i in 1:sample_rate:length(qs)]
    joint_vels = [vs[i][n-1] for i in 1:sample_rate:length(vs)]
    paths[string("qs", n)] = joint_poses
    paths[string("vs", n)] = joint_vels
end

# function plot_state_errors()
    l = @layout [a b ; c d ; e f ; g h]
    # label = ["q2", "q3", "v2", "v3"]
    # Joint E (base joint)
    p1 = plot(ts_down, paths["qs8"], label="Joint E", ylim=(-1.5, 1.5))
    # p1 = plot!(LinRange(0,duration,50), poses[:,1], label="des_q2")
    p2 = plot(ts_down, paths["vs8"], label="Joint E vels",  ylim=(-0.5, 0.5))
    # p3 = plot!(LinRange(0, duration, 50), vels[:,1], label="des_v2")
    # Joint D (shoulder joint)
    p3 = plot(ts_down, paths["qs9"], label="Joint D",  ylim=(-1.5, 1.5))
    # p2 = plot!(LinRange(0, duration, 50), poses[:,2], label="des_q3")
    p4 = plot(ts_down, paths["vs9"], label="Joint D vels",  ylim=(-0.5, 0.5))
    # p4 = plot!(LinRange(0, duration, 50), vels[:,2], label="des_v3")
    # Joint C (elbow joint)
    p5 = plot(ts_down, paths["qs10"], label="Joint C",  ylim=(-1.5, 1.5))
    # p2 = plot!(LinRange(0, duration, 50), poses[:,2], label="des_q3")
    p6 = plot(ts_down, paths["vs10"], label="Joint C vels",  ylim=(-0.5, 0.5))
    # p4 = plot!(LinRange(0, duration, 50), vels[:,2], label="des_v3")
    # Joint B (wrist joint)
    p7 = plot(ts_down, paths["qs11"], label="Joint B",  ylim=(-1.5, 1.5))
    # p2 = plot!(LinRange(0, duration, 50), poses[:,2], label="des_q3")
    p8 = plot(ts_down, paths["vs11"], label="Joint B vels",  ylim=(-0.5, 0.5))
    # p4 = plot!(LinRange(0, duration, 50), vels[:,2], label="des_v3")
    plot(p1, p2, p3, p4, p5, p6, p7, p8, layout=l)
# end

#%% Animate
# render(mvis)
# MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 0.5)