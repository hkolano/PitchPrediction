# ----------------------------------------------------------
#                     Import Libraries
# ----------------------------------------------------------
#%%
using RigidBodyDynamics
using LinearAlgebra, StaticArrays, DataStructures
using MeshCat, MeshCatMechanisms, MechanismGeometries
using CoordinateTransformations
using GeometryBasics
using Printf, Plots, CSV, Tables, ProgressBars
using PitchPrediction

# include("/home/hkolano/onr-dynamics-julia/simulate_with_ext_forces.jl")


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

delete!(vis)

# Create visuals of the URDFs
visuals = URDFVisuals(urdf_file)
mvis = MechanismVisualizer(mech_sea_alpha, URDFVisuals(urdf_file), vis[:alpha])
render(mvis)
#%%

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
og_link_drag_coeffs = [drag_link1, drag_link2, drag_link3, drag_link4]

og_d_lin_coeffs = [4.5, 8.0, 2.7, 3.4, 4.6, 52.9]
og_d_nonlin_coeffs = [11.4, 20.0, 13.5, 34.4, 65.9, 132.3]

println("CoM and CoB frames initialized. \n")

# ----------------------------------------------------------
#                 State Initialization
# ----------------------------------------------------------
#%%

function randomize_link_drag(og_link_drag_coeffs, percent_error)
    link_drag_coeffs = [zeros(3), zeros(3), zeros(3), zeros(3)]
    for n = 1:4
        for m = 1:3
            d = og_link_drag_coeffs[n][m]
            link_drag_coeffs[n][m] = randomize_value(d, percent_error) 
        end
    end
    return link_drag_coeffs
end

function randomize_inertia(inertia, percent_error)
    new_mass = randomize_value(inertia.mass, percent_error)
    new_ixx = randomize_value(inertia.moment[1,1], percent_error)
    new_iyy = randomize_value(inertia.moment[2,2], percent_error)
    new_izz = randomize_value(inertia.moment[3,3], percent_error)
    new_ixy = randomize_value(inertia.moment[1,2], percent_error)
    new_ixz = randomize_value(inertia.moment[1,3], percent_error)
    new_iyz = randomize_value(inertia.moment[2,3], percent_error)
    new_moment = [new_ixx new_ixy new_ixz; new_ixy new_iyy new_iyz; new_ixz new_iyz new_izz]
    new_inertia = SpatialInertia(inertia.frame,
        SMatrix{3, 3, Float64}(new_moment),
        SVector{3, Float64}(inertia.cross_part),
        Float64(new_mass))
    return new_inertia
end

function randomize_all_added_masses(mechanism, percent_error)
    # println("Randomizing added masses")
    for body in bodies(mechanism)[2:end]
        # println("Randomizing body: $(body)")
        spatial_inertia!(body, randomize_inertia(spatial_inertia(body), percent_error))
    end
end

function randomize_value(value, percent)
    low_range = 1-percent
    high_range = 1+percent 
    if abs(value) >= 0.002
        new_value = rand(low_range*value:.001:high_range*value)
    else
        new_value = 0.0
    end
    return new_value
end

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
per_error = 0.01

#%%
# (temporary adds while making changes to ctlr and traj generator)
include(ctlr_file)
include(traj_file)

# ----------------------------------------------------------
#                      Gather Sim Data
# ----------------------------------------------------------

num_trajs = 2

# Create (num_trajs) different trajectories and save to csvs
for n in ProgressBar(1:num_trajs)

    global d_lin_coeffs = [randomize_value(d, per_error) for d in og_d_lin_coeffs]
    global d_nonlin_coeffs = [randomize_value(d, per_error) for d in og_d_nonlin_coeffs]
    global link_drag_coeffs = randomize_link_drag(og_link_drag_coeffs, per_error)
    randomize_all_added_masses(mech_sea_alpha, per_error)

    # Reset the sim to the equilibrium position
    reset_to_equilibrium!(state)
    # Start up the controller
    ctlr_cache = PIDCtlr.CtlrCache(Δt, mech_sea_alpha)

    # ----------------------------------------------------------
    #                          Simulate
    # ----------------------------------------------------------
    # Generate a random waypoint and see if there's a valid trajectory to it
    wp = TrajGen.gen_rand_waypoints_from_equil()
    traj = TrajGen.find_trajectory(wp)

    # Keep trying until a good trajectory is found
    while traj === nothing
        wp = TrajGen.gen_rand_waypoints_from_equil()
        traj = TrajGen.find_trajectory(wp)
    end

    # Scale that trajectory to 1x-3x "top speed"
    scaled_traj = TrajGen.scale_trajectory(traj...)
    params = scaled_traj[1]
    duration = scaled_traj[2]
    poses = scaled_traj[3]
    vels = scaled_traj[4]

    # Make vector of waypoint values and time step to save to csv
    waypoints = [Δt*sample_rate params.wp.start.θs... params.wp.goal.θs... params.wp.start.dθs... params.wp.goal.dθs...]
    wp_data = Tables.table(waypoints)

    # Save waypoints (start and goal positions, velocities) to CSV file
    if n == 1
        goal_headers = ["dt", "E_start", "D_start", "C_start", "B_start", "E_end", "D_end", "C_end", "B_end", "dE_start", "dD_start", "dC_start", "dB_start", "dE_end", "dD_end", "dC_end", "dB_end"]
        CSV.write("data/full-sim-with-hydro/point1percent-error/waypoints_082322.csv", wp_data, header=goal_headers)
    else 
        CSV.write("data/full-sim-with-hydro/point1percent-error/waypoints_082322.csv", wp_data, header=false, append=true)
    end

    ts, qs, vs = simulate_with_ext_forces(state, duration, params, ctlr_cache, hydro_calc!, PIDCtlr.pid_control!; Δt=Δt)
    # ts, qs, vs = simulate(state_alpha, final_time, simple_control!; Δt = 1e-2)

    ts_down = [ts[i] for i in 1:sample_rate:length(ts)]
    des_vs = [TrajGen.get_desv_at_t(t, params) for t in ts_down]
    paths = OrderedDict();

    paths["qs0"] = [qs[i][1] for i in 1:sample_rate:length(qs)]
    for idx = 1:10
        joint_poses = [qs[i][idx+1] for i in 1:sample_rate:length(qs)]
        paths[string("qs", idx)] = joint_poses
    end
    for idx = 1:10
        joint_vels = [vs[i][idx] for i in 1:sample_rate:length(vs)]
        paths[string("vs", idx)] = joint_vels
    end

    # Write the trajectories to an array -> Table -> CSV
    num_rows = 25
    data = Array{Float64}(undef, length(ts_down), num_rows-4)
    fill!(data, 0.0)
    labels = Array{String}(undef, num_rows-4)

    quat_data = Array{Float64}(undef, length(ts_down), 4)
    quat_labels = Array{String}(undef, 4)
    row_n = 1
    for (key, value) in paths
        if row_n < 5
            quat_labels[row_n] = key 
            quat_data[:,row_n] = value
        else
            labels[row_n-4] = key
            data[:,row_n-4] = value 
        end
        row_n = row_n + 1
    end
    for actuated_idx = 5:8
        # println([des_vs[m][actuated_idx] for m in 1:length(des_vs)])
        data[:,row_n-4] = [des_vs[m][actuated_idx] for m in 1:length(des_vs)]
        row_n = row_n + 1
    end
    labels[18:21] = ["des_vsE", "des_vsD", "des_vsC", "des_vsB"]
    
    tab = Tables.table(data)
    CSV.write("data/full-sim-with-hydro/point1percent-error/data-no-orientation/states$(n).csv", tab, header=labels)
    quat_tab = Tables.table(quat_data)
    CSV.write("data/full-sim-with-hydro/point1percent-error/data-quat/quats$(n).csv", quat_tab, header=quat_labels)

    # MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 1.0)
end

println("Simulation finished.")


# function plot_state_errors()
    # l = @layout [a b ; c d ; e f]
    # # label = ["q2", "q3", "v2", "v3"]
    # # Joint E (base joint)
    # p1 = plot(ts_down, paths["qs8"], label="Joint E", ylim=(-3.0, 3.0))
    # p1 = plot!(LinRange(0,duration,50), poses[:,1], label="des_qE", legend=:topleft)
    # p2 = plot(ts_down, paths["vs8"], label="Joint E vels",  ylim=(-0.5, 0.5))
    # p2 = plot!(LinRange(0, duration, 50), vels[:,1], label="des_vE", legend=:topleft)
    # # Joint D (shoulder joint)
    # p3 = plot(ts_down, paths["qs9"], label="Joint D",  ylim=(-.5, 5.5))
    # p3 = plot!(LinRange(0, duration, 50), poses[:,2], label="des_qD", legend=:topleft)
    # p4 = plot(ts_down, paths["vs9"], label="Joint D vels",  ylim=(-0.5, 0.5))
    # p4 = plot!(LinRange(0, duration, 50), vels[:,2], label="des_vD", legend=:topleft)
    # # Joint C (elbow joint)
    # p5 = plot(ts_down, paths["qs10"], label="Joint C",  ylim=(-.5, 5.5))
    # p5 = plot!(LinRange(0, duration, 50), poses[:,3], label="des_qC", legend=:topleft)
    # p6 = plot(ts_down, paths["vs10"], label="Joint C vels",  ylim=(-0.5, 0.5))
    # p6 = plot!(LinRange(0, duration, 50), vels[:,3], label="des_vC", legend=:topleft)
    # # Joint B (wrist joint)
    # # p7 = plot(ts_down, paths["qs11"], label="Joint B",  ylim=(-1.5, 1.5))
    # # p7 = plot!(LinRange(0, duration, 50), poses[:,4], label="des_q3", legend=:topleft)
    # # p8 = plot(ts_down, paths["vs11"], label="Joint B vels",  ylim=(-0.5, 0.5))
    # # p8 = plot!(LinRange(0, duration, 50), vels[:,4], label="des_v3", legend=:topleft)
    # # plot(p1, p2, p3, p4, p5, p6, p7, p8, layout=l)
    # display(plot(p1, p2, p3, p4, p5, p6, layout=l))
# end

# for n = 1:6
#     veh_vels = [vs[i][n] for i in 1:sample_rate:length(vs)]
#     paths[string("vs", n)] = veh_vels
# end

# l2 = @layout [a b ; c d]
# # label = ["q2", "q3", "v2", "v3"]
# p_yaw = plot(ts_down, paths["vs3"], label="twist - yaw", ylim=(-.5, .5))
# p_surge = plot(ts_down, paths["vs4"], label="twist - x",  ylim=(-0.5, 0.5))
# p_sway = plot(ts_down, paths["vs5"], label="twist - y",  ylim=(-.5, .5))
# p_heave = plot(ts_down, paths["vs6"], label="twist - z",  ylim=(-0.5, 0.5))
# plot(p_yaw, p_surge, p_sway, p_heave, layout=l2)

#  Animate
# render(mvis)
