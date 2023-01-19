# ----------------------------------------------------------
#                     Import Libraries
# ----------------------------------------------------------
#%%
using RigidBodyDynamics
using LinearAlgebra, StaticArrays, DataStructures
using MeshCat, MeshCatMechanisms, MechanismGeometries
using CoordinateTransformations
using GeometryBasics
using Printf, Plots, CSV, Tables, ProgressBars, Revise


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
~, vehicle_body, shoulder_body, upper_arm_body, elbow_body, wrist_body, jaw_body = bodies(mech_blue_alpha)

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
    set_configuration!(state, vehicle_joint, [.9777, -.0019, 0.2098, .0079, 0., 0., 0.])
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
include("HydroCalc.jl")
include("SimWExt.jl")


# ----------------------------------------------------------
#                      Gather Sim Data
# ----------------------------------------------------------

num_trajs = 1 
save_to_csv = false
show_animation = true
plot_velocities = true
plot_control_taus = true

# Create (num_trajs) different trajectories and save to csvs 
# for n in ProgressBar(1:num_trajs)

    # Reset the sim to the equilibrium position
    reset_to_equilibrium!(state)
    # Start up the controller
    ctlr_cache = PIDCtlr.CtlrCache(Δt, mech_blue_alpha)
    # ctlr_cache.taus[:,1] = [0.; 0.; 0.; 0.; 0.; 10.; 0.; 0.; 0.; 0.]

    # ----------------------------------------------------------
    #                          Simulate
    # ----------------------------------------------------------
    # Generate a random waypoint and see if there's a valid trajectory to it
    # wp = TrajGen.gen_rand_waypoints_to_rest()
    wp = TrajGen.gen_rand_waypoints_from_equil()
    # wp = TrajGen.load_waypoints("pid_test")
    # wp = TrajGen.set_waypoints_from_equil([-2.03, 1.92, 1.73, 1.18], [0.16, 0.24, -0.08, 0.19])

    traj = TrajGen.find_trajectory(wp) 

    # # Keep trying until a good trajectory is found
    while traj === nothing
        global wp = TrajGen.gen_rand_waypoints_to_rest()
        global traj = TrajGen.find_trajectory(wp)
    end

    # # Scale that trajectory to 1x-3x "top speed"
    if do_scale_traj == true
        scaled_traj = TrajGen.scale_trajectory(traj...)
    else
        scaled_traj = traj 
    end
    params = scaled_traj[1]
    duration = params.T
    poses = scaled_traj[2]
    vels = scaled_traj[3]

    # Make vector of waypoint values and time step to save to csv
    waypoints = [Δt*sample_rate params.wp.start.θs[1:4]... params.wp.goal.θs[1:4]... params.wp.start.dθs[1:4]... params.wp.goal.dθs[1:4]...]
    wp_data = Tables.table(waypoints)

    # Save waypoints (start and goal positions, velocities) to CSV file
    if save_to_csv == true
        if n == 1
            goal_headers = ["dt", "E_start", "D_start", "C_start", "B_start", "E_end", "D_end", "C_end", "B_end", "dE_start", "dD_start", "dC_start", "dB_start", "dE_end", "dD_end", "dC_end", "dB_end"]
            CSV.write("data/full-sim-data-110822/full-sim-waypoints_110822.csv", wp_data, header=goal_headers)
        else 
            CSV.write("data/full-sim-data-110822/full-sim-waypoints_110822.csv", wp_data, header=false, append=true)
        end
    end

    # Simulate the trajectory
    if save_to_csv != true; println("Simulating... ") end
    ts, qs, vs = simulate_with_ext_forces(state, duration+duration_after_traj, params, ctlr_cache, hydro_calc!, PIDCtlr.pid_control!; Δt=Δt)
    # ts, qs, vs = simulate_with_ext_forces(state, .002, params, ctlr_cache, hydro_calc!, PIDCtlr.pid_control!; Δt=Δt)
    if save_to_csv != true; println("done.") end

    # Downsample the desired velocities
    ts_down = [ts[i] for i in 1:sample_rate:length(ts)]
    des_vs = [TrajGen.get_desv_at_t(t, params) for t in ts_down]
    paths = OrderedDict();

    # Downsample the simulation output
    paths["qs0"] = [qs[i][1] for i in 1:sample_rate:length(qs)]
    for idx = 1:10
        joint_poses = [qs[i][idx+1] for i in 1:sample_rate:length(qs)]
        paths[string("qs", idx)] = joint_poses
    end
    for idx = 1:10
        joint_vels = [vs[i][idx] for i in 1:sample_rate:length(vs)]
        paths[string("vs", idx)] = joint_vels
    end

    if show_animation == true
        print("Animating... ")
        MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 1.0)
        println("done.")
    end

    if plot_velocities == true
        print("Plotting...")
        l = @layout[a b; c d; e f]
        var_names = ["vs1", "vs2", "vs3", "vs4", "vs5", "vs6"]
        plot_labels = ["roll", "pitch", "yaw", "x", "y", "z"]
        plot_handles = []
        for k = 1:6
            var = var_names[k]
            lab = plot_labels[k]
            if k < 4
                push!(plot_handles, plot(ts_down, paths[var], title=lab, legend=false, titlefontsize=12))
            else
                push!(plot_handles, plot(ts_down, paths[var], title=lab, ylim=(-.05,.05), legend=false, titlefontsize=12))
            end
        end
        display(plot(plot_handles..., layout=l))
        println("done.")
    end

    if plot_control_taus == true
        tau_plot_handles = []
        tau_plot_lims = [[-3, 3], [-6, 0], [-3, 3], [4, 10]]
        tl = @layout[a b; c d]
        for k = 3:6
            lab = plot_labels[k]
                push!(tau_plot_handles, plot(ctlr_cache.taus[k,2:end], title=lab, legend=false, ylim=tau_plot_lims[k-2]))
        end
        display(plot(tau_plot_handles..., layout=tl))
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
# end
# function plot_state_errors()
#     l = @layout [a b ; c d ; e f]
#     # label = ["q2", "q3", "v2", "v3"]
#     # Joint E (base joint)
#     p1 = plot(ts_down, paths["qs8"], label="Joint E", ylim=(-3.0, 3.0))
#     p1 = plot!(LinRange(0,duration,50), poses[:,1], label="des_qE", legend=:topleft)
#     p2 = plot(ts_down, paths["vs8"], label="Joint E vels",  ylim=(-0.5, 0.5))
#     p2 = plot!(LinRange(0, duration, 50), vels[:,1], label="des_vE", legend=:topleft)
#     # Joint D (shoulder joint)
#     p3 = plot(ts_down, paths["qs9"], label="Joint D",  ylim=(-.5, 5.5))
#     p3 = plot!(LinRange(0, duration, 50), poses[:,2], label="des_qD", legend=:topleft)
#     p4 = plot(ts_down, paths["vs9"], label="Joint D vels",  ylim=(-0.5, 0.5))
#     p4 = plot!(LinRange(0, duration, 50), vels[:,2], label="des_vD", legend=:topleft)
#     # Joint C (elbow joint)
#     p5 = plot(ts_down, paths["qs10"], label="Joint C",  ylim=(-.5, 5.5))
#     p5 = plot!(LinRange(0, duration, 50), poses[:,3], label="des_qC", legend=:topleft)
#     p6 = plot(ts_down, paths["vs10"], label="Joint C vels",  ylim=(-0.5, 0.5))
#     p6 = plot!(LinRange(0, duration, 50), vels[:,3], label="des_vC", legend=:topleft)
#     # Joint B (wrist joint)
#     # p7 = plot(ts_down, paths["qs11"], label="Joint B",  ylim=(-1.5, 1.5))
#     # p7 = plot!(LinRange(0, duration, 50), poses[:,4], label="des_q3", legend=:topleft)
#     # p8 = plot(ts_down, paths["vs11"], label="Joint B vels",  ylim=(-0.5, 0.5))
#     # p8 = plot!(LinRange(0, duration, 50), vels[:,4], label="des_v3", legend=:topleft)
#     # plot(p1, p2, p3, p4, p5, p6, p7, p8, layout=l)
#     display(plot(p1, p2, p3, p4, p5, p6, layout=l))
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