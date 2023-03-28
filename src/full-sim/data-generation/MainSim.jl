#= 
Main flight code for running the dynamics simulations.
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

include("HydroCalc.jl")
include("SimWExt.jl")
include("PIDCtlr.jl")
include("UVMSPlotting.jl")
include("HelperFuncs.jl")
include("Noiser.jl")

include("UVMSsetup.jl")
include("ConfigFiles/MagicNumPitchPred.jl")
include("ConfigFiles/ConstMagicNums.jl")
include("ConfigFiles/MagicNumBlueROV.jl")
include("ConfigFiles/MagicNumAlpha.jl")

urdf_file = joinpath("urdf", "blue_rov.urdf")

# ----------------------------------------------------------
#                 One-Time Mechanism Setup
# ----------------------------------------------------------
mech_blue_alpha, mvis, joint_dict, body_dict = mechanism_reference_setup(urdf_file)
include("TrajGenJoints.jl")

cob_frame_dict, com_frame_dict = setup_frames(body_dict, body_names, cob_vec_dict, com_vec_dict)
buoyancy_force_dict, gravity_force_dict = setup_buoyancy_and_gravity(buoyancy_mag_dict, grav_mag_dict)

state = MechanismState(mech_blue_alpha)
num_dofs = num_velocities(mech_blue_alpha)
num_actuated_dofs = num_dofs-2
#%%
# ----------------------------------------------------------
#                      Gather Sim Data
# ----------------------------------------------------------
# Control variables
num_trajs = 1
save_to_csv = false
show_animation = false
bool_plot_velocities = true
bool_plot_taus = true
bool_plot_positions = false

# Create (num_trajs) different trajectories and save to csvs 
# for n in ProgressBar(1:num_trajs)
include("TrajGenJoints.jl")
    # Create trajectory 
    params = quinticTrajParams[]
    swap_times = Vector{Float64}()
    define_multiple_waypoints!(params, swap_times, 2)
    println("Scaled trajectory duration: $(swap_times[end]) seconds")

#%%
include("PIDCtlr.jl")
include("Noiser.jl.")
    # Reset the sim to the equilibrium position
    reset_to_equilibrium!(state)
    # Start up the controller
    noise_cache = NoiseCache(state)
    filter_cache = FilterCache(state)
    ctlr_cache = CtlrCache(state, noise_cache, filter_cache)

    # Simulate the trajectory
    if save_to_csv != true; println("Simulating... ") end
    ts, qs, vs = simulate_with_ext_forces(state, swap_times[end], params, ctlr_cache, hydro_calc!, pid_control!; Δt=Δt)
    # ts, qs, vs = simulate_with_ext_forces(state, 20, params, ctlr_cache, hydro_calc!, pid_control!; Δt=Δt)
    if save_to_csv != true; println("done.") end

    @show vs[end]'
#%%
    # Downsample the time steps to goal_freq
    ts_down = [ts[i] for i in 1:sample_rate:length(ts)]
    ts_down_no_zero = ts_down[2:end]

    # Set up data collection dicts
    paths = prep_actual_vels_and_qs_for_plotting()
    des_paths = prep_desired_vels_and_qs_for_plotting()
    
    meas_paths, filt_paths = prep_data_for_plotting()

    
    qs_noisy = Array{Float64}(undef, length(ts_down_no_zero), 10)
    for i = 1:length(ts_down_no_zero)
        qs_noisy[i,1:3] = convert_to_rpy(ctlr_cache.n_cache.noisy_qs[1:4, i])
        qs_noisy[i,4:end] = ctlr_cache.n_cache.noisy_qs[5:end,i]
    end
    
    for idx=1:10 
        # Positions
        meas_paths[string("qs", idx)] = qs_noisy[:,idx]
    end
    for idx = 1:10
        # Velocities
        meas_paths[string("vs", idx)] = [ctlr_cache.n_cache.noisy_vs[idx,Int(i*plot_factor)] for i in 1:length(ts_down_no_zero)]
        filt_paths[string("vs", idx)] = [ctlr_cache.f_cache.filtered_vs[idx,Int(i*plot_factor)] for i in 1:length(ts_down_no_zero)]
    end

    include("UVMSPlotting.jl")

    if bool_plot_velocities == true
        plot_des_vs_act_velocities(ts_down_no_zero, 
            paths, des_paths, meas_paths, filt_paths, 
            plot_veh=false, plot_arm=true)
    end

    if bool_plot_positions == true
        plot_des_vs_act_positions(ts_down_no_zero, des_ts, 
            paths, des_paths, meas_paths, 
            plot_veh = true, plot_arm=true)
    end

    # if bool_plot_taus == true
    #     plot_control_taus(ctlr_cache, ts_down)
    # end 

    # Use stop_step to visualize specific points in the trajectory
    # stop_step = 29*1000
    if show_animation == true
        print("Animating... ")
        # MeshCatMechanisms.animate(mvis, ts[1:stop_step], qs[1:stop_step]; realtimerate = 5.0)
        MeshCatMechanisms.animate(mvis, ts, qs; realtimerate=1.0)
        println("done.")
    end

    # only save the trajectory if Joint 1 doesn't exceed the joint velocity limits (which is a proxy for indicating whether it is unstable)
    if save_to_csv == true && maximum(paths["vs7"]) < 0.9
        # Rows:
        # 1-10: Actual position data (qs)
        # 11-20: Actual velocity data (vs)
        # 21-30: Noisy position data (noisy_qs)
        # 31-40: Noisy velocity data (noisy_vs)
        # 41-44: Desired velocities 
        num_rows = 44
        data = Array{Float64}(undef, length(ts_down_no_zero), num_rows)
        fill!(data, 0.0)
        labels = Array{String}(undef, num_rows)

        row_n = 1
        for (key, value) in paths
            labels[row_n] = key
            data[:,row_n] = value 
            row_n = row_n + 1
        end  
        for (key, value) in meas_paths
            # @show key
            labels[row_n] = "meas_"*key 
            data[:,row_n] = value 
            row_n = row_n + 1
        end
        for (key, value) in des_paths_same_ts
            # @show key
            labels[row_n] = "des_"*key 
            data[:,row_n] = value
            row_n = row_n + 1
        end
        tab = Tables.table(data)
        println("Saving trajectory...")
        CSV.write("data/full-sim-data-022323/states$(n).csv", tab, header=labels)
        # to save an example trajectory to plot in MATLAB:
        # CSV.write("data/full-sim-data-022223/example_traj.csv", tab, header=labels)
    else
        println("Not saving trajectory")
    end
    println("")
# end