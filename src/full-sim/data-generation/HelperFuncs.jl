using Rotations

function convert_to_rpy(quat_vals)
    quat_rot = QuatRotation(quat_vals...)
    euler = RotXYZ(quat_rot)
    vals = [euler.theta1, euler.theta2, euler.theta3] 
end

function save_traj_to_csv(num_rows::Int)
    data = Array{Float64}(undef, length(ts_down_no_zero), num_rows)
    fill!(data, 0.0)
    labels = Array{String}(undef, num_rows)

    row_n = 1
    row_n = write_dict_to_data!(data, labels, row_n, paths, "")
    row_n = write_dict_to_data!(data, labels, row_n, meas_paths, "meas_")
    
    arm_positions_keys = des_paths.keys[1:Int(end/2)]
    des_paths_vs = deepcopy(des_paths)
    # @show des_paths_vs
    for pos_key in arm_positions_keys
        delete!(des_paths_vs, pos_key)
    end
    row_n = write_dict_to_data!(data, labels, row_n, des_paths_vs, "des_")

    tab = Tables.table(data)
    println("Saving trajectory...")
    CSV.write("data/temp_data/states.csv", tab, header=labels)
    # to save an example trajectory to plot in MATLAB:
    # CSV.write("data/full-sim-data-022223/example_traj.csv", tab, header=labels)
end

function write_dict_to_data!(data, labels, row_n::Int, dict::OrderedDict, prefix::String)
    for (key, value) in dict 
        labels[row_n] = prefix*key 
        @show labels[row_n]
        data[:,row_n] = value 
        row_n = row_n + 1
    end 
    return row_n
end