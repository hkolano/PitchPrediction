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

include("FrameSetup.jl")
include("HydroCalc.jl")
include("SimWExt.jl")
include("PIDCtlr.jl")
include("TrajGenJoints.jl")
include("UVMSPlotting.jl")
include("HelperFuncs.jl")

urdf_file = joinpath("urdf", "blue_rov.urdf")

#%%
# ----------------------------------------------------------
#                 One-Time Mechanism Setup
# ----------------------------------------------------------
vis = Visualizer()
mech_blue_alpha = parse_urdf(urdf_file; floating=true, gravity = [0.0, 0.0, 0.0])
# mech_blue_alpha = parse_urdf(urdf_file; gravity = [0.0, 0.0, 0.0])

delete!(vis)

# Create visuals of the URDFs
mvis = MechanismVisualizer(mech_blue_alpha, URDFVisuals(urdf_file), vis[:alpha])
# render(mvis)

# Name the joints and bodies of the mechanism
# vehicle_joint, base_joint, shoulder_joint, elbow_joint, wrist_joint, jaw_joint = joints(mech_blue_alpha)
# world, vehicle_body, shoulder_body, upper_arm_body, elbow_body, wrist_body, jaw_body = bodies(mech_blue_alpha)
vehicle_joint, base_joint, shoulder_joint, elbow_joint, wrist_joint = joints(mech_blue_alpha)
world, vehicle_body, shoulder_body, upper_arm_body, elbow_body, wrist_body = bodies(mech_blue_alpha)

body_frame = default_frame(vehicle_body)
shoulder_frame = default_frame(shoulder_body)
upper_arm_frame = default_frame(upper_arm_body)
elbow_frame = default_frame(elbow_body)
wrist_frame = default_frame(wrist_body)
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
com_vecs = [SVector{3, Float64}([0.0, 0.0, 0.0]), SVector{3, Float64}([0.005, -.001, 0.016]), SVector{3, Float64}([0.073, 0.0, 0.0]), SVector{3, Float64}([0.017, -0.026, -0.002]), SVector{3, Float64}([0.0, 0.0, -.098]), SVector{3, Float64}([0.0, 0.0, 0.0])]
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
# KEEP ARM BASE VALUES AT END OF LIST for HydroCalc
rho = 997
volumes = [10.23/(.001*rho), .018, .203, .025, .155, .202] # vehicle, shoulder, ua, elbow, wrist, jaw, armbase
buoy_force_mags = volumes * rho * 9.81 * .001
buoy_lin_forces = []
for mag in buoy_force_mags
    lin_force = FreeVector3D(base_frame, [0.0, 0.0, mag])
    push!(buoy_lin_forces, lin_force)
end
# println(buoy_lin_forces)

masses = [10.0, .194, .429, .115, .333, .341]
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

function reset_to_equilibrium!(state)
    zero!(state)
    set_configuration!(state, vehicle_joint, [.9777, -0.0019, 0.2098, .0079, 0., 0., 0.])
    # set_velocity!(state, vehicle_joint, [0., 0., 0., 0., 0.0, 0.])
end

# Constants
state = MechanismState(mech_blue_alpha)
Δt = 1e-3
ctrl_freq = 100
final_time = 5.0
goal_freq = 100
sample_rate = Int(floor((1/Δt)/goal_freq))

# Control variables
do_scale_traj = true   # Scale the trajectory?
duration_after_traj = 1.0   # How long to simulate after trajectory has ended

#%%
# (temporary adds while making changes to ctlr and traj generator)
include("PIDCtlr.jl")
include("TrajGenJoints.jl")
include("HydroCalc.jl")
include("SimWExt.jl")

# wp = TrajGen.generate_path_from_current_pose(state)
# println("Starting Pose")
# println(wp.start_pose)
# println("Goal Pose")
# println(wp.end_pose)
# traj_params = TrajGen.find_trajectory(wp)

# des_pose, des_vel = TrajGen.get_des_state_at_t(0.1, wp, traj_params[1])



# p_arm = get_ee_path(mech_blue_alpha, jaw_body)

# ----------------------------------------------------------
#                      Gather Sim Data
# ----------------------------------------------------------

num_trajs = 1 
save_to_csv = false
show_animation = false
bool_plot_velocities = true
bool_plot_taus = true
bool_plot_positions = false

# Create (num_trajs) different trajectories and save to csvs 
# for n in ProgressBar(1:num_trajs)

   
    # ctlr_cache.taus[:,1] = [0.; 0.; 0.; 0.; 0.; 10.; 0.; 0.; 0.; 0.]
 
    # ----------------------------------------------------------
    #                          Simulate
    # ----------------------------------------------------------
    # Generate a random waypoint and see if there's a valid trajectory to it
    wp = gen_rand_waypoints_to_rest()
    # wp = gen_rand_waypoints_from_equil()
    # wp = TrajGen.load_waypoints("pid_test")
    # wp = TrajGen.set_waypoints_from_equil([-2.03, 1.92, 1.73, 1.18], [0.16, 0.24, -0.08, 0.19])

    traj = find_trajectory(wp) 

    # # Keep trying until a good trajectory is found
    while traj === nothing
        global wp = gen_rand_waypoints_to_rest()
        global traj = find_trajectory(wp)
    end

    # a, T, des_poses, des_vels = find_trajectory(wp)
    # goal_frame = wp.end_pose.from
    # add_frame!(world, wp.end_pose)
    # setelement!(mvis, goal_frame)
    # visualize_path(des_poses, mvis, world)
    # traj_pars = trajParams(a, wp, T)

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
    println("Scaled trajectory duration: $(duration) seconds")
    poses = scaled_traj[2]
    vels = scaled_traj[3]

    #%%
    include("PIDCtlr.jl")
    include("HydroCalc.jl")
    include("TrajGenJoints.jl")
    # Reset the sim to the equilibrium position
    reset_to_equilibrium!(state)
    # Start up the controller
    ctlr_cache = CtlrCache(Δt, ctrl_freq, state)
    # Simulate the trajectory
    if save_to_csv != true; println("Simulating... ") end
    # ts, qs, vs = simulate_with_ext_forces(state, duration+duration_after_traj, params, ctlr_cache, hydro_calc!, pid_control!; Δt=Δt)
    ts, qs, vs = simulate_with_ext_forces(state, 10, params, ctlr_cache, hydro_calc!, pid_control!; Δt=Δt)
    if save_to_csv != true; println("done.") end

    # Downsample the time steps to goal_freq
    ts_down = [ts[i] for i in 1:sample_rate:length(ts)]
    ts_down_no_zero = ts_down[2:end]

    paths = OrderedDict();
    des_paths = OrderedDict();
    meas_paths = OrderedDict();
    filt_paths = OrderedDict();

    # Calculate desired velocities
    des_vs = [get_desv_at_t(t, params) for t in ts_down_no_zero]
    des_qs = [get_desq_at_t(t, params) for t in ts_down_no_zero]

    qs_down = Array{Float64}(undef, length(ts_down_no_zero), 10)
    vs_down = zeros(length(ts_down_no_zero), 10)
    qs_noisy = Array{Float64}(undef, length(ts_down_no_zero), 10)
    ct = 1
    i = 1
    while ct <= length(ts_down_no_zero)
        qs_down[ct, 1:3] = convert_to_rpy(qs[i][1:4])
        qs_down[ct,4:end] = qs[i][5:end]
        vs_down[ct,:] = vs[i]
        ct += 1
        i += sample_rate
    end
    for i = 1:length(ts_down_no_zero)
        qs_noisy[i,1:3] = convert_to_rpy(ctlr_cache.noisy_qs[1:4, i])
        qs_noisy[i,4:end] = ctlr_cache.noisy_qs[5:end,i]
    end
    
    for idx = 1:10
        # Velocities
        paths[string("vs", idx)] = vs_down[:,idx]
        des_paths[string("vs", idx)] = idx > 2 ? [des_vs[i][idx-2] for i in 1:length(ts_down_no_zero)] : zeros(length(ts_down_no_zero))
        meas_paths[string("vs", idx)] = [ctlr_cache.noisy_vs[idx,i] for i in 1:length(ts_down_no_zero)]
        filt_paths[string("vs", idx)] = [ctlr_cache.filtered_vs[idx,i] for i in 1:length(ts_down_no_zero)]
        
        # Positions
        paths[string("qs", idx)] = qs_down[:,idx] 
        des_paths[string("qs", idx)] = idx > 2 ? [des_qs[i][idx-2] for i in 1:length(ts_down_no_zero)] : zeros(length(ts_down_no_zero))
        meas_paths[string("qs", idx)] = qs_noisy[:,idx]
    end

    include("UVMSPlotting.jl")

    if bool_plot_velocities == true
        plot_des_vs_act_velocities(ts_down_no_zero, 
            paths, des_paths, meas_paths, filt_paths, 
            plot_veh=true, plot_arm=true)
    end

    if bool_plot_positions == true
        plot_des_vs_act_positions(ts_down_no_zero,
            paths, des_paths, meas_paths, 
            plot_veh = true, plot_arm=true)
    end

    if bool_plot_taus == true
        plot_control_taus(ctlr_cache, ts_down)
    end 

    if show_animation == true
        print("Animating... ")
        MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 1.0)
        println("done.")
    end

    if save_to_csv == true
        # Rows:
        # 1-10: Actual position data (qs)
        # 11-20: Actual velocity data (vs)
        # 21-30: Noisy position data (noisy_qs)
        # 31-40: Noisy velocity data (noisy_vs)
        # 41-44: Desired velocities for arm
        num_rows = 44
        data = Array{Float64}(undef, length(ts_down), num_rows)
        fill!(data, 0.0)
        labels = Array{String}(undef, num_rows)

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
        CSV.write("data/full-sim-data-021623/states$(n).csv", tab, header=labels)
    end