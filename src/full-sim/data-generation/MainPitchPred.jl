#= 
Main flight code for running the Pitch Prediction pipeline.
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
simhelperfuncsfile = joinpath("..", "hinsdale_post_processing", "simcomparisonfuncs.jl")

urdf_file = joinpath("urdf", "blue_rov_hardware_fixedjaw.urdf")


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

#%%
# ----------------------------------------------------------
#                   Start: Gather Sim Data
# ----------------------------------------------------------
# Control variables
num_trajs = 5000
save_to_csv = true
show_plots = false
show_animation = false

# Create (num_trajs) different trajectories and save to csvs 
for n in ProgressBar(1369:num_trajs)

    # ----------------------------------------------------------
    #                   Define a Trajectory
    # ----------------------------------------------------------
    params, _, _ = define_random_trajectory()
    println("Scaled trajectory duration: $(params.T) seconds")

# include("ConfigFiles/MagicNumPitchVal.jl")
# include("PIDCtlr.jl")
    # ----------------------------------------------------------
    #                  Setup and Run Simulation
    # ----------------------------------------------------------
    # Reset the sim to the equilibrium position
    zero!(state)
    set_configuration!(state, joint_dict["vehicle"], [.9993, .0339, .0124, -.004, 0., 0., 0.])
    # set_configuration!(state, joint_dict["vehicle"], [.99956, .01133, -.0269, -.004, 0., 0., 0])
    set_configuration!(state, joint_dict["base"], params.wp.start.θs[1])
    set_configuration!(state, joint_dict["shoulder"], params.wp.start.θs[2])
    set_configuration!(state, joint_dict["elbow"], params.wp.start.θs[3])
    set_configuration!(state, joint_dict["wrist"], params.wp.start.θs[4])

    # Start up the controller
    noise_cache = NoiseCache(state)
    filter_cache = FilterCache(state)
    ctlr_cache = CtlrCache(state, noise_cache, filter_cache)
 
    start_buffer = rand(10:.01:18)
    end_buffer = rand(2:.01:10)
    delayed_params = delayedQuinticTrajParams(params,start_buffer, params.T+start_buffer)
    println("Total trajectory time will be: $(params.T+start_buffer+end_buffer) seconds")

    # Simulate the trajectory
    try
        global ts, qs, vs = simulate_with_ext_forces(state, delayed_params.endbuffer+end_buffer, delayed_params, ctlr_cache, hydro_calc!, pid_control!; Δt=Δt)
        show_plots = true
        save_to_csv = true
    catch y
        println("Errors during simulation. Skipping to next trajectory.")
        show_plots = false 
        save_to_csv = false
        # ts, qs, vs = simulate_with_ext_forces(state, 12, delayed_params, ctlr_cache, hydro_calc!, pid_control!; Δt=Δt)
    end
    # @show vs[end]'

    # ----------------------------------------------------------
    #                      Prepare Plots
    # ----------------------------------------------------------
    if show_plots == true
        gr(size=(800, 600)) 
        # Downsample the time steps to goal_freq
        ts_down = [ts[i] for i in 1:sample_rate:length(ts)]
        global ts_down_no_zero = ts_down[2:end]

        include("UVMSPlotting.jl")

        sim_palette = palette([:deepskyblue2, :magenta], 4)
        actual_palette = palette([:goldenrod1, :springgreen3], 4)

        # Set up data collection dicts
        paths = prep_actual_vels_and_qs_for_plotting(ts_down_no_zero)
        sim_df = DataFrame(paths)
        sim_df[!,:time_secs] = ts_down_no_zero
        des_paths = prep_desired_vels_and_qs_for_plotting(ts_down_no_zero, delayed_params)
        des_df = DataFrame(des_paths)
        # meas_paths = prep_measured_vels_and_qs_for_plotting()
        # filt_paths = prep_filtered_vels_for_plotting()

        deleteat!(des_df, findall(<(10), des_df[!,:time_secs]))
        deleteat!(sim_df, findall(<(10), sim_df[!,:time_secs]))
    
    
        p_ori = new_plot()
        xaxis!(p_ori, grid = (:x, :solid, .75, .9), minorgrid = (:x, :dot, .5, .5))
        @df sim_df plot!(p_ori, :time_secs, [:qs1, :qs2], 
            palette=sim_palette, linewidth=2, linestyle=:dash, 
            label=["sim roll" "sim pitch"])
        plot!(p_ori, legend=:outerbottomright)
        ylabel!("Vehicle Orientation (rad)")
        title!("BlueROV Orientation")


        p_js = new_plot()
        xaxis!(p_js, grid = (:x, :solid, .75, .9), minorgrid = (:x, :dot, .5, .5))
        @df des_df plot!(p_js, :time_secs, 
            [:qs7.+3.07, :qs8, :qs9, :qs10.+2.879]; 
            palette=:atlantic, linewidth=2, linestyle=:solid, 
            label=["des axis e" "des axis d" "des axis c" "des axis b"])
        @df sim_df plot!(p_js, :time_secs, 
            [cols(7).+3.07, cols(8), cols(9), cols(10).+2.879]; #, cols(11)]; 
            palette=sim_palette, linewidth=2, linestyle=:dash, 
            label=["sim axis e" "sim axis d" "sim axis c" "sim axis b"])
        plot!(p_js, legend=:outerbottomright)
        ylabel!("Joint position (rad)")
        title!("Alpha Arm Joint Positions")
        plot!(p_js, ylims=(-.5, 6))

        super_plot = plot(p_js, p_ori, layout=(2, 1), plot_title="Train Data Number $(n)")
        display(super_plot)
    end

    # if bool_plot_taus == true
    #     plot_control_taus(ctlr_cache, ts_down)
    # end 

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
        deleteat!(sim_df, 1:2:length(sim_df[!,:time_secs]))
        sim_df[!,:time_secs] = sim_df[!,:time_secs] .- minimum(sim_df[!,:time_secs])
        new_file_name = joinpath("data", "full-sim-data-091023", "train", lpad(n, 4, "0")*".csv")
        CSV.write(new_file_name, sim_df)
    else
        println("Not saving trajectory")
    end
    println("")
end