#= 
Main flight code for running the pitch prediciton pipeline with the setup used in Hinsdale trials.
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

using DataFrames, StatsPlots, Interpolations

include("HydroCalc.jl")
include("SimWExt.jl")
include("PIDCtlr.jl")
include("UVMSPlotting.jl")
include("HelperFuncs.jl")
include("Noiser.jl")

include("UVMSsetup.jl")
include("ConfigFiles/MagicNumPitchVal.jl")
include("ConfigFiles/ConstMagicNums.jl")
include("ConfigFiles/MagicNumBlueROVHardware.jl")
include("ConfigFiles/MagicNumAlpha.jl")
trajparsingfile = joinpath("..", "hinsdale_post_processing", "gettrajparamsfromyaml.jl")
interpolationfile = joinpath("..", "hinsdale_post_processing", "mocap_interpolation.jl")
simhelperfuncsfile = joinpath("..", "hinsdale_post_processing", "simcomparisonfuncs.jl")
include(trajparsingfile)
include(interpolationfile)


urdf_file = joinpath("urdf", "blue_rov_hardware.urdf")

# ----------------------------------------------------------
#                 One-Time Mechanism Setup
# ----------------------------------------------------------
mech_blue_alpha, mvis, joint_dict, body_dict = mechanism_reference_setup(urdf_file)
include("TrajGenJoints.jl")
include(simhelperfuncsfile)


cob_frame_dict, com_frame_dict = setup_frames(body_dict, body_names, cob_vec_dict, com_vec_dict)
buoyancy_force_dict, gravity_force_dict = setup_buoyancy_and_gravity(buoyancy_mag_dict, grav_mag_dict)

state = MechanismState(mech_blue_alpha)
num_dofs = num_velocities(mech_blue_alpha)
num_actuated_dofs = num_dofs-2

# ----------------------------------------------------------
#                 Get Data for Comparison
# ----------------------------------------------------------
trial_code = "004-0"
# trial_code = "baseline1"

# sim_offset = 1
params, des_df, sim_offset = gettrajparamsfromyaml(trial_code, "fullrange2")

# Get mocap data 
mocap_df = get_vehicle_response_from_csv(trial_code, "hinsdale-data-2023", "fullrange2")
imu_df = get_imu_data_from_csv(trial_code, "hinsdale-data-2023")
# 
avg_qs_at_offset, init_vs, init_ωs = get_initial_conditions(0, mocap_df)
init_vs_vector = FreeVector3D(root_frame(mech_blue_alpha), init_vs)
body_frame_init_vs = RigidBodyDynamics.transform(state, init_vs_vector, default_frame(body_dict["vehicle"]))


# ----------------------------------------------------------
#                         Simulate
# ----------------------------------------------------------
#%%
    include("HydroCalc.jl")
    # ----------------------------------------------------------
    #                  Setup and Run Simulation
    # ----------------------------------------------------------
    include("PIDCtlr.jl")
    # Give the vehicle initial conditions from the mocap
    zero!(state)
    set_configuration!(state, joint_dict["vehicle"], avg_qs_at_offset)
    set_velocity!(state, joint_dict["vehicle"], [0, 0, 0, body_frame_init_vs.v...])

    # set_configuration!(state, joint_dict["vehicle"], [.9239, 0, 0, 0.382, 0.5, 0., 0.])
    # Start up the controller
    noise_cache = NoiseCache(state)
    filter_cache = FilterCache(state)
    ctlr_cache = CtlrCache(state, noise_cache, filter_cache)

    start_buffer = sim_offset
    end_buffer = 7.5
    delayed_params = delayedQuinticTrajParams(params,start_buffer, params.T+start_buffer)

    # Simulate the trajectory
    if save_to_csv != true; println("Simulating... ") end
    ts, qs, vs = simulate_with_ext_forces(state, params.T+start_buffer+end_buffer, delayed_params, ctlr_cache, hydro_calc!, pid_control!; Δt=Δt)
    # ts, qs, vs = simulate_with_ext_forces(state, 10, params, ctlr_cache, hydro_calc!, pid_control!; Δt=Δt)
    # ts, qs, vs = simulate_with_ext_forces(state, .5, delayed_params, ctlr_cache, hydro_calc!, pid_control!; Δt=Δt)
    if save_to_csv != true; println("done.") end

    @show vs[end]'
#%%
    # ----------------------------------------------------------
    #                      Prepare Plots
    # ----------------------------------------------------------
    include("UVMSPlotting.jl")
    gr(size=(800, 800)) 
    @show sim_offset

    sim_palette = palette([:deepskyblue2, :magenta], 4)
    actual_palette = palette([:goldenrod1, :springgreen3], 4)

    js_df = get_js_data_from_csv(trial_code, "hinsdale-data-2023")
    
    imu_df = calc_rpy(imu_df)
    # mocap_df = get_vehicle_response_from_csv(trial_code, "hinsdale-data-notimetrim", "fullrange2")
    # real_start_time = minimum([mocap_df[1,1], js_df[1,1]])
    # mocap_df[!,:time_secs] = mocap_
    # if real_start_time != 0.0
    #     mocap_df[!,:time_secs] = mocap_df[!,:time_secs].-real_start_time
    #     js_df[!,:time_secs] = js_df[!,:time_secs].-real_start_time
    # end

    # Downsample the time steps to goal_freq
    ts_down = [ts[i] for i in 1:sample_rate:length(ts)]
    ts_down_no_zero = ts_down[2:end]

    # # Set up data collection dicts
    paths = prep_actual_vels_and_qs_for_plotting()
    sim_df = DataFrame(paths)
    sim_df[!,"time_secs"] = ts_down_no_zero
    # meas_paths = prep_measured_vels_and_qs_for_plotting()
    # filt_paths = prep_filtered_vels_for_plotting()

    p_zed = new_plot()
    @df mocap_df plot!(p_zed, :time_secs, [:z_pose, :y_pose, :x_pose]; :goldenrod1, linewidth=2, label=["mocap z" "mocap_y" "mocap_x"])
    # @df mocap_df plot!(p_zed, :time_secs[1:2500], [:z_pose[1:2500], :y_pose[1:2500], :x_pose[1:2500]]; :goldenrod1, linewidth=2, label=["mocap z" "mocap_y" "mocap_x"])
    @df sim_df plot!(p_zed, :time_secs, [:qs6, :qs5, :qs4]; :deepskyblue2, linewidth=2, linestyle=:dash, label=["sim z" "sim y" "sim x"])
    title!(p_zed, "Vehicle Position for trial "*trial_code)
    ylabel!(p_zed, "Position (m)")
    plot!(p_zed, legend=:outerbottomright)
    label=["actual x_ori" "actual y_ori" "actual z_ori" "actual w_ori"]
    xaxis!(p_zed, grid = (:x, :solid, .75, .9), minorgrid = (:x, :dot, .5, .5))


    # p_quats = new_plot()
    # @df mocap_df plot!(p_quats, :time_secs[1:2500], 
    #     [:x_ori[1:2500], :y_ori[1:2500], :z_ori[1:2500], :w_ori[1:2500]], 
    #     palette=actual_palette, linewidth=2, 
    #     label=["actual x_ori" "actual y_ori" "actual z_ori" "actual w_ori"])
    # xaxis!(p_quats, grid = (:x, :solid, .75, .9), minorgrid = (:x, :dot, .5, .5))
    # @df sim_df plot!(p_quats, :time_secs.+sim_offset, 
    #     [:x_ori, :y_ori, :z_ori, :w_ori], 
    #     palette=sim_palette, linewidth=2, linestyle=:dash, 
    #     label=label=["sim x_ori" "sim y_ori" "sim z_ori" "sim w_ori"])
    # plot!(p_quats, legend=:outerbottomright)
    # title!("BlueROV Quaternion")
    # ylabel!("Quaternion value")

    # artificial_offset = -1.
    artificial_offset = 0

    p_vehrp = new_plot()
    # @df mocap_df plot!(p_vehrp, :time_secs[1:3200], [:roll[1:3200], :pitch[1:3200]], palette=actual_palette, linewidth=2, label=["actual roll" "actual pitch"])
    @df mocap_df plot!(p_vehrp, :time_secs, [:roll, :pitch], palette=actual_palette, linewidth=2, label=["mocap roll" "mocap pitch"])
    xaxis!(p_vehrp, grid = (:x, :solid, .75, .9), minorgrid = (:x, :dot, .5, .5))
    @df sim_df plot!(p_vehrp, :time_secs.+artificial_offset, [:qs1, :qs2], 
        palette=sim_palette, linewidth=2, linestyle=:dash, 
        label=["sim roll" "sim pitch"])
        plot!(p_vehrp, legend=:outerbottomright)
    @df imu_df plot!(p_vehrp, :time_secs, [:roll, :pitch], linewidth=2, label=["imu roll" "imu pitch"])
    ylabel!("Vehicle Orientation (rad)")
    title!("BlueROV Orientation")

    @show rad2deg(get_pitch_rmse(imu_df, sim_df))
    @show rad2deg(get_pitch_rmse(imu_df, sim_df, true, -1.))

    # super_ori_plot = plot(p_zed, p_vehrp, layout=(2,1), plot_title="Comparison for "*trial_code)

    p_js = new_plot()
    @df js_df plot!(p_js, :time_secs, cols(3:6); palette=actual_palette, linewidth=2)
    xaxis!(p_js, grid = (:x, :solid, .75, .9), minorgrid = (:x, :dot, .5, .5))
    # @df des_df plot!(p_js, :time_secs, cols(2:5); palette=:grayC, linewidth=2, linestyle=:dash)
    @df sim_df plot!(p_js, :time_secs, 
        [cols(7).+3.07, cols(8), cols(9), cols(10).+1.57]; #, cols(11)]; 
        palette=sim_palette, linewidth=2, linestyle=:dash, 
        label=["sim axis e" "sim axis d" "sim axis c" "sim axis b"])
    plot!(p_js, legend=:outerbottomright)
    ylabel!("Joint position (rad)")
    title!("Alpha Arm Joint Positions")
    plot!(p_js, ylims=(-.5, 6))
    # if bool_plot_velocities == true
    #     plot_des_vs_act_velocities(ts_down_no_zero, 
    #         paths, des_paths, meas_paths, filt_paths, 
    #         plot_veh=false, plot_arm=true)
    # end

    # if bool_plot_positions == true
    #     plot_des_vs_act_positions(ts_down_no_zero, des_ts, 
    #         paths, des_paths, meas_paths, 
    #         plot_veh = true, plot_arm=true)
    # end

    # if bool_plot_taus == true
    #     plot_control_taus(ctlr_cache, ts_down)
    # end 

    super_plot = plot(p_js, p_vehrp, layout=(2, 1), plot_title="Sim vs Hinsdale, traj "*trial_code*" (artificial offset "*string(artificial_offset)*"s)")
#%%
    # ----------------------------------------------------------
    #                  Animate the Trajectory
    # ----------------------------------------------------------
    # Use stop_step to visualize specific points in the trajectory
    # stop_step = 29*1000
    if show_animation == true
        print("Animating... ")
        # MeshCatMechanisms.animate(mvis, ts[1:stop_step], qs[1:stop_step]; realtimerate = 5.0)
        MeshCatMechanisms.animate(mvis, ts, qs; realtimerate=1.)
        println("done.")
    end

    # ----------------------------------------------------------
    #                 Save Trajectory to CSV
    # ----------------------------------------------------------
    # only save the trajectory if Joint 1 doesn't exceed the joint velocity limits (which is a proxy for indicating whether it is unstable)
    if save_to_csv == true && maximum(paths["vs7"]) < 0.9
        # Rows:
        # 1-10: Actual position data (qs)
        # 11-20: Actual velocity data (vs)
        # 21-30: Noisy position data (noisy_qs)
        # 31-40: Noisy velocity data (noisy_vs)
        # 41-44: Desired velocities 
        include("HelperFuncs.jl")
        num_rows = 44
        save_traj_to_csv(num_rows)
    else
        println("Not saving trajectory")
    end
    println("")
# end