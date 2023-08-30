using Interpolations, DataFrames
#TODO look into DataInterpolations (Akima interpolation?)

function get_initial_conditions(interp_start_time, df; dt=.01)
    # Set up evenly spaced time intervals
    IC_times = interp_start_time:dt:interp_start_time+.05
    even_ts = interp_start_time:dt:df[end-1, :time_secs]

    init_xyz = mean(Array(trimmed_mocap_df[1:5, 2:4]), dims=1)

    itp_dict = Dict()
    init_qs = Vector{Float64}(undef, 7)
    init_vs = Vector{Float64}(undef, 3)
    q_names = ["w_ori", "x_ori", "y_ori", "z_ori", "x_pose", "y_pose", "z_pose"]
    qs_window = Dict()

    for (idx, meas_name) in enumerate(q_names)
        itp_dict[meas_name] = interpolate((Array(df[!, :time_secs]),), Array(df[!, meas_name]), Gridded(Linear()))
        qs_window[meas_name] = itp_dict[meas_name](IC_times)
        init_qs[idx] = mean(qs_window[meas_name])
    end

    lin_vel_names = q_names[5:7]
    for (idx, meas_name) in enumerate(lin_vel_names)
        num_interped_vels = length(qs_window[meas_name])-2
        interped_vels = Vector{Float64}(undef, num_interped_vels)
        for i in 1:num_interped_vels
            interped_vels[i] = (qs_window[meas_name][i+2] - qs_window[meas_name][i])/2dt
        end
        init_vs[idx] = mean(interped_vels)
    end


    return init_qs, init_vs

end