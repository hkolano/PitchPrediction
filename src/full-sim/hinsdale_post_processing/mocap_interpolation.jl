using DataFrames, DataInterpolations, Smoothers, StatsBase
#TODO look into DataInterpolations (Akima interpolation?)

function interp_at_timesteps(timestamps, df, col_names, type="akima")
    itp_dict = Dict()
    interped_df = DataFrame()
    interped_df[!,:time_secs] = timestamps
    for (idx, meas_name) in enumerate(col_names)
        if "type" == "akima"
            itp_dict[meas_name] = DataInterpolations.AkimaInterpolation(Array(df[!, meas_name]), Array(df[!, :time_secs]))
        else
            itp_dict[meas_name] = DataInterpolations.LinearInterpolation(Array(df[!, meas_name]), Array(df[!, :time_secs]))
        end
            # itp_dict[meas_name] = interpolate((Array(df[!, :time_secs]),), Array(df[!, meas_name]), Gridded(Linear()))
        interped_df[!, meas_name] = itp_dict[meas_name](timestamps)
    end

    return interped_df
end


function get_pitch_rmse(mocap_df, sim_df, with_offset=false, a_offset=0)  
    col_names = ["w_ori", "x_ori", "y_ori", "z_ori"]
    mocap_end_time = mocap_df[end,:time_secs]
    sim_time_steps = sim_df[!,:time_secs]
    trimmed_sim_time_steps = filter(x->x<mocap_end_time, sim_time_steps)
    num_timesteps = length(trimmed_sim_time_steps)

    interped_mocap_df = interp_at_timesteps(trimmed_sim_time_steps, mocap_df, col_names)
    interped_mocap_df = calc_rpy(interped_mocap_df)

    if with_offset == false
        rmse = rmsd(interped_mocap_df[!,:pitch], sim_df[1:num_timesteps,:qs2])
    else
        num_offset_steps = -Int(a_offset/.02)
        rmse = rmsd(interped_mocap_df[!,:pitch][1:end-num_offset_steps], sim_df[1:num_timesteps,:qs2][num_offset_steps+1:end])
    end
    return rmse
end

function get_initial_vehicle_velocities(interp_start_time, df; dt=.01) 
    init_vs = Vector{Float64}(undef, 3)
    init_qs = Vector{Float64}(undef, 3)
    lin_vel_names = ["x_pose", "y_pose", "z_pose"]
    IC_times = interp_start_time:dt:interp_start_time+.1

    for (idx, meas_name) in enumerate(lin_vel_names)
        # create interpolation object for this measurement
        itp = DataInterpolations.AkimaInterpolation(Array(df[!, meas_name]), Array(df[!, :time_secs]))
        # interpolate on several time points close to the start time
        qs_window = itp(IC_times)
        num_interped_vels = length(qs_window)-2
        interped_vels = Vector{Float64}(undef, num_interped_vels)
        # Do finite differencing between poses to get velocities
        for i in 1:num_interped_vels
            interped_vels[i] = (qs_window[i+2] - qs_window[i])/2dt
        end
        init_qs[idx] = mean(qs_window)
        init_vs[idx] = mean(interped_vels)
    end
    return init_vs, init_qs
end

function get_initial_conditions(interp_start_time, df; dt=.01)
    # Set up evenly spaced time intervals
    IC_times = interp_start_time-.03:dt:interp_start_time+.03

    itp_dict = Dict()
    init_qs = Vector{Float64}(undef, 4)

    q_names = ["w_ori", "x_ori", "y_ori", "z_ori"]
    smoothed_mocap_df = DataFrame()
    qs_window = Dict()

    for meas_name in q_names 
        smoothed_mocap_df[!,meas_name] = sma(df[!,meas_name], 100)
    end

    for (idx, meas_name) in enumerate(q_names)
        itp_dict[meas_name] = DataInterpolations.AkimaInterpolation(Array(df[!, meas_name]), Array(df[!, :time_secs]))
        # itp_dict[meas_name] = interpolate((Array(df[!, :time_secs]),), Array(df[!, meas_name]), Gridded(Linear()))
        qs_window[meas_name] = itp_dict[meas_name](IC_times)
        init_qs[idx] = mean(qs_window[meas_name])
    end

    num_interped_ang_vels = length(qs_window["w_ori"])-1
    ang_vel_list = Array{Float64}(undef, num_interped_ang_vels, 3)
    for i in 1:num_interped_ang_vels
        quat1 = QuatRotation(qs_window["w_ori"][i], qs_window["x_ori"][i], qs_window["y_ori"][i], qs_window["z_ori"][i])
        quat2 = QuatRotation(qs_window["w_ori"][i+1], qs_window["x_ori"][i+1], qs_window["y_ori"][i+1], qs_window["z_ori"][i+1])
        ang_vel_list[i,:] = ang_vel_from_quaternions(quat1, quat2, dt)
    end
    ang_vels = mean(ang_vel_list, dims=1)
    return init_qs, ang_vels
end

function ang_vel_from_quaternions(q1::QuatRotation, q2::QuatRotation, dt)
    ωx = (2/dt)*(q1.w*q2.x - q1.x*q2.w - q1.y*q2.z + q1.z*q2.y)
    ωy = (2/dt)*(q1.w*q2.y + q1.x*q2.z - q1.y*q2.w - q1.z*q2.x)
    ωz = (2/dt)*(q1.w*q2.z - q1.x*q2.y + q1.y*q2.x - q1.z*q2.w)
    return [ωx, ωy, ωz]
end