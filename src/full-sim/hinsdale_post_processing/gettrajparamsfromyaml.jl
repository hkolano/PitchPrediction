using CSV, YAML
using MeshCat, MeshCatMechanisms, MechanismGeometries
using RigidBodyDynamics

function gettrajparamsfromyaml(trial_code, dataset="fullrange2")

    # Load desired traj data 
    if dataset == "fullrange2"
        path_to_data = joinpath("src", "full-sim", "data-generation", "combo_traj_yaml_files_"*dataset, "traj"*trial_code[1:3], "traj"*trial_code*".yaml")
    else
        path_to_data = joinpath("src", "full-sim", "data-generation", "combo_traj_yaml_files_"*dataset, "traj"*trial_code[6:8], "traj"*trial_code[6:end]*".yaml")
    end
    des_data = YAML.load_file(path_to_data)

    start_state = jointState(reverse(des_data["start_pos"]), reverse(des_data["start_vel"]))
    end_state = jointState(reverse(des_data["end_pos"]), reverse(des_data["end_vel"]))
    pts = Waypoints(start_state, end_state)
    duration = des_data["duration"]

    num_its = 200
    poses = Array{Float64}(undef, num_its, num_trajectory_dofs)
    vels = Array{Float64}(undef, num_its, num_trajectory_dofs)
    a = Array{Float64}(undef, num_trajectory_dofs, 6)
    time_secs = Array{Float64}(undef, num_its)

    # reconstruct trajectory
    for i in 1:num_trajectory_dofs
        dof_name = dof_names[i+6]
        a[i,:] = get_coeffs(pts, duration, i)
        if i != 5
            (poses[:,i], vels[:,i]) = get_path!(poses[:,i], vels[:,i], pts.start.θs[i], pts.goal.θs[i], duration, a[i,:], num_its)
        end
    end
    fill!(poses[:,5], 0.0)
    fill!(vels[:,5], 0.0)

    dt = duration/num_its

    for i = 1:num_its
        time_secs[i] = dt*i
    end

    metadata_file_loc = joinpath("data", "hinsdale-data-2023", "metadata_"*trial_code*".yml")
    alignment = YAML.load_file(metadata_file_loc)
    offset = alignment["start_time"]

    des_df = DataFrame(time_secs=time_secs.+offset, b_des_poses=poses[:,4], c_des_poses=poses[:,3], d_des_poses=poses[:,2], e_des_poses=poses[:,1])
    this_traj_params = quinticTrajParams(a, pts, duration)
    return (this_traj_params, des_df, offset)
end


function get_vehicle_response_from_csv(trial_code, foldername="hinsdale-data-2023", dataset="fullrange2")
    if trial_code[1] == '0'
        mocap_datapath = joinpath("data", foldername, "traj"*trial_code*"_mocap.csv")
    else 
        mocap_datapath = joinpath("data", foldername, trial_code*"_mocap.csv")
    end
        # mocap_datapath = joinpath("data", foldername, trial_code*"_mocap.csv")
    mocap_df = CSV.read(mocap_datapath, DataFrame)
    dropmissing!(mocap_df)
    return mocap_df
end

function get_js_data_from_csv(trial_code, foldername="hinsdale-data-2023", dataset="fullrange2")
    js_filepath = joinpath("data", foldername, "traj"*trial_code*"_joint_states.csv")
    # js_filepath = joinpath("data", "hinsdale-data-2023", trial_code*"_joint_states.csv")
    js_df = CSV.read(js_filepath, DataFrame)
    dropmissing!(js_df)
    return js_df
end