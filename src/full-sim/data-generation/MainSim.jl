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
using JLD, Random

# include("/home/hkolano/onr-dynamics-julia/simulate_with_ext_forces.jl")
#%%
include("StructDefs.jl")
include("FrameSetup.jl")
include("HydroCalc.jl")
include("SimWExt.jl")
include("PIDCtlr.jl")
include("TrajGenMain.jl")

urdf_file = joinpath("urdf", "blue_rov.urdf")

# ----------------------------------------------------------
#                 One-Time Mechanism Setup
# ----------------------------------------------------------
#%%
vis = Visualizer()
mech_blue_alpha = parse_urdf(urdf_file; floating=true, gravity = [0.0, 0.0, 0.0])
# mech_blue_alpha = parse_urdf(urdf_file; gravity = [0.0, 0.0, 0.0])

delete!(vis)

# Create visuals of the URDFs
mvis = MechanismVisualizer(mech_blue_alpha, URDFVisuals(urdf_file), vis[:alpha])
# render(mvis)

# Name the joints and bodies of the mechanism
vehicle_joint, base_joint, shoulder_joint, elbow_joint, wrist_joint, jaw_joint = joints(mech_blue_alpha)
world, vehicle_body, shoulder_body, upper_arm_body, elbow_body, wrist_body, jaw_body = bodies(mech_blue_alpha)

body_frame = default_frame(vehicle_body)
shoulder_frame = default_frame(shoulder_body)
upper_arm_frame = default_frame(upper_arm_body)
elbow_frame = default_frame(elbow_body)
wrist_frame = default_frame(wrist_body)
jaw_frame = default_frame(jaw_body)
base_frame = root_frame(mech_blue_alpha)
# setelement!(mvis_alpha, shoulder_frame)

println("Mechanism built.")

# ----------------------------------------------------------
#                 COM and COB Frame Setup
# ----------------------------------------------------------
frame_names_cob = ["vehicle_cob", "shoulder_cob", "ua_cob", "elbow_cob", "wrist_cob", "jaw_cob"]
frame_names_com = ["vehicle_com", "shoulder_com", "ua_com", "elbow_com", "wrist_com", "jaw_com"]
# Assume default frame = COM
cob_vecs = [SVector{3, Float64}([0.0, 0.0, 0.02]), SVector{3, Float64}([-0.001, -0.003, .032]), SVector{3, Float64}([0.073, 0.0, -0.002]), SVector{3, Float64}([0.003, 0.001, -0.017]), SVector{3, Float64}([0.0, 0.0, -.098]), SVector{3, Float64}([0.0, 0.0, 0.0])]
com_vecs = [SVector{3, Float64}([0.0, 0.0, 0.0]), SVector{3, Float64}([0.005, -.001, 0.016]), SVector{3, Float64}([0.073, 0.0, 0.0]), SVector{3, Float64}([0.017, -0.026, -0.002]), SVector{3, Float64}([0.0, 0.003, -.098]), SVector{3, Float64}([0.0, 0.0, 0.0])]
cob_frames = []
com_frames = []
setup_frames!(mech_blue_alpha, frame_names_cob, frame_names_com, cob_vecs, com_vecs, cob_frames, com_frames)

#%%
# buoyancy force setup
# ---------------------------------------------------------------
#                       BUOYANCY SETUP
# ---------------------------------------------------------------
# f = rho * g * V
# f = 997 (kg/m^3) * 9.81 (m/s^2) * V_in_L *.001 (m^3) = kg m / s^2
# One time setup of buoyancy forces
rho = 997
volumes = [10.23/(.001*rho), .018, .203, .025, .155, 0.01, .202] # vehicle, shoulder, ua, elbow, wrist, jaw, armbase
buoy_force_mags = volumes * rho * 9.81 * .001
buoy_lin_forces = []
for mag in buoy_force_mags
    lin_force = FreeVector3D(base_frame, [0.0, 0.0, mag])
    push!(buoy_lin_forces, lin_force)
end
# println(buoy_lin_forces)

masses = [10.0, .194, .429, .115, .333, 0.01, .341]
grav_forces = masses*9.81
grav_lin_forces = []
for f_g in grav_forces
    lin_force = FreeVector3D(base_frame, [0.0, 0.0, -f_g])
    push!(grav_lin_forces, lin_force)
end
# println(grav_lin_forces)

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

function reset_to_equilibrium!(state)
    zero!(state)
    # set_configuration!(state, vehicle_joint, [.9777, -.0019, 0.2098, .0079, 0., 0., 0.])
    set_configuration!(state, vehicle_joint, [.6533, .2706, -.2706, .6533, 0, 0, 0])
    # set_velocity!(state, vehicle_joint, [0., 0., 0., 0., 0.0, 0.])
end

# Constants
state = MechanismState(mech_blue_alpha)
Δt = 1e-3
final_time = 5.0
goal_freq = 50
sample_rate = Int(floor((1/Δt)/goal_freq))

# Control variables
do_scale_traj = true   # Scale the trajectory?
duration_after_traj = 1.0   # How long to simulate after trajectory has ended

#%%
# (temporary adds while making changes to ctlr and traj generator)
include("PIDCtlr.jl")
include("TrajGenMain.jl")
# include("HydroCalc.jl")
# include("SimWExt.jl")
include("Tasks.jl")


p_arm = get_ee_path(mech_blue_alpha, jaw_body)

# ----------------------------------------------------------
#                      Gather Sim Data
# ----------------------------------------------------------

num_trajs = 1 
save_to_csv = false
show_animation = true
plot_poses = true
plot_zetas_bool = true
plot_joint_config_bool = true
plot_control_taus_bool = true

# Create (num_trajs) different trajectories and save to csvs 
# for n in ProgressBar(1:num_trajs)

    # Reset the sim to the equilibrium position
    reset_to_equilibrium!(state)
    # Start up the controller
    ctlr_cache = CtlrCache(Δt, mech_blue_alpha)
    # ctlr_cache.taus[:,1] = [0.; 0.; 0.; 0.; 0.; 10.; 0.; 0.; 0.; 0.]

    # ----------------------------------------------------------
    #                          Simulate
    # ----------------------------------------------------------
    # Generate a random waypoint and see if there's a valid trajectory to it
    # wp = TrajGen.gen_rand_waypoints_to_rest()
    wp = gen_rand_waypoints_from_equil()
    # wp = TrajGen.load_waypoints("pid_test")
    # wp = TrajGen.set_waypoints_from_equil([-2.03, 1.92, 1.73, 1.18], [0.16, 0.24, -0.08, 0.19])

    traj = find_trajectory(wp) 

    # # Keep trying until a good trajectory is found
    while traj === nothing
        global wp = gen_rand_waypoints_to_rest()
        global traj = find_trajectory(wp)
    end

    a, T, des_poses, des_vels = find_trajectory(wp)
    goal_frame = wp.end_pose.from
    add_frame!(world, wp.end_pose)
    setelement!(mvis, goal_frame)
    visualize_path(des_poses, mvis, world)
    traj_pars = trajParams(a, wp, T)

    # include("kinematicsandbox.jl")

    # qs = typeof(configuration(state))[]
    # reset_to_equilibrium!(state)

    # for t in range(0, stop=1, length=1000)
    #     jacobian_transpose_ik!(state, traj_pars, ctlr_cache, t)
    #     push!(qs, copy(configuration(state)))
    # end
    # ts = collect(range(0, stop=1, length=length(qs)))
    # setanimation!(vis, Animation(vis, ts, qs))


    # # Scale that trajectory to 1x-3x "top speed"
    if do_scale_traj == true
        scaled_traj = scale_trajectory(traj...)
    else
        scaled_traj = traj 
    end
    params = scaled_traj[1]
    duration = params.T
    poses = scaled_traj[2]
    vels = scaled_traj[3]

    # Save waypoints (start and goal positions, velocities) to CSV file
    if save_to_csv == true
        # Make vector of waypoint values and time step to save to csv
        waypoints = [Δt*sample_rate params.wp.start.θs[1:4]... params.wp.goal.θs[1:4]... params.wp.start.dθs[1:4]... params.wp.goal.dθs[1:4]...]
        wp_data = Tables.table(waypoints)
        if n == 1
            goal_headers = ["dt", "E_start", "D_start", "C_start", "B_start", "E_end", "D_end", "C_end", "B_end", "dE_start", "dD_start", "dC_start", "dB_start", "dE_end", "dD_end", "dC_end", "dB_end"]
            CSV.write("data/full-sim-data-110822/full-sim-waypoints_110822.csv", wp_data, header=goal_headers)
        else 
            CSV.write("data/full-sim-data-110822/full-sim-waypoints_110822.csv", wp_data, header=false, append=true)
        end
    end

    # Simulate the trajectory
    if save_to_csv != true; println("Simulating... ") end
    # ts, qs, vs = simulate_with_ext_forces(state, T+duration_after_traj, traj_pars, ctlr_cache, hydro_calc!, pid_control!; Δt=Δt)
    ts, qs, vs = simulate_with_ext_forces(state, 6, traj_pars, ctlr_cache, hydro_calc!, pid_control!; Δt=Δt)
    if save_to_csv != true; println("done.") end

    # Downsample the desired velocities
    ts_down = [ts[i] for i in 1:sample_rate:length(ts)]

    if show_animation == true
        print("Animating... ")
        MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 1.0)
        println("done.")
    end

    include("UVMSPlotting.jl")

    if plot_poses == true
        plot_desired_and_actual_poses(traj_pars, qs, ts_down)
    end

    if plot_zetas_bool == true
        plot_zetas(ctlr_cache, vs, ts_down)
    end

    if plot_joint_config_bool == true
        plot_joint_config(qs, ts_down)
    end

    if plot_control_taus_bool == true
        plot_control_taus(ctlr_cache, ts_down)
    end

    if save_to_csv == true
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
        CSV.write("data/full-sim-data-110822/data-no-orientation/states$(n).csv", tab, header=labels)
        quat_tab = Tables.table(quat_data)
        CSV.write("data/full-sim-data-110822/data-quat/quats$(n).csv", quat_tab, header=quat_labels)
    end