using MeshCat, MeshCatMechanisms, MechanismGeometries
using RigidBodyDynamics
using Glob

import YAML

include("UVMSsetup.jl")
include("ConfigFiles/MagicNumInvKin.jl")

urdf_file = joinpath("urdf", "blue_rov_hardware.urdf")
mech_blue_alpha, mvis, joint_dict, body_dict = mechanism_reference_setup(urdf_file)

include("TrajGenJoints.jl")

#%%

path_to_data = joinpath("src", "full-sim", "data-generation", "combo_traj_yaml_files")
traj_file_names = readdir(path_to_data)

for file_name in traj_file_names
    # check that file still exists
    file_path = joinpath(path_to_data, file_name)
    if isfile(file_path) == false
        continue
    end

    # get trajectory data from file
    println("Parsing data file ", file_name)
    data = YAML.load_file(file_path)
    start_state = jointState(reverse(data["start_pos"]), reverse(data["start_vel"]))
    end_state = jointState(reverse(data["end_pos"]), reverse(data["end_vel"]))
    pts = Waypoints(start_state, end_state)
    duration = data["duration"]

    num_its=10

    # set up memory for pose, velocity lists
    poses = Array{Float64}(undef, num_its, num_trajectory_dofs)
    vels = Array{Float64}(undef, num_its, num_trajectory_dofs)
    a = Array{Float64}(undef, num_trajectory_dofs, 6)

    state = MechanismState(mech_blue_alpha)
    zero!(state)
    qs = typeof(configuration(state))[]

    # reconstruct trajectory
    for i in 1:num_trajectory_dofs
        dof_name = dof_names[i+6]
        a[i,:] = get_coeffs(pts, duration, i)
        (poses[:,i], vels[:,i]) = get_path!(poses[:,i], vels[:,i], pts.start.θs[i], pts.goal.θs[i], duration, a[i,:], num_its)
    end

    for i in 1:num_its
        new_state_qs = copy(configuration(state))
        new_state_qs[8:11] = poses[i,1:4]
        set_configuration!(state, new_state_qs)
        push!(qs, copy(configuration(state)))
    end

    ts = collect(range(0, stop=duration, length=length(qs)))
    MeshCatMechanisms.animate(mvis, ts, qs; realtimerate=4.)

    finished_with_traj = false
    println("Trajectory completed. ")

    while finished_with_traj == false
        println("Keep (k), trash (t), or rewatch (r) trajectory? You may have to enter it twice.")
        # key = IJulia.readprompt("prompt")
        key = readline()

        if key == "k"
            finished_with_traj = true
            println("Keeping file; showing next trajectory.")
        elseif key == "t"
            finished_with_traj = true 
            
            # file_series = string(file_name[1:end-7], "*.yaml")
            traj_name = file_name[end-13:end-7]
            glob_path = string(path_to_data, "/", traj_name, "*.yaml")
            # println("Glob path is:")
            # println(glob_path)
            println("Deleting whole trajectory series: ")
            for file in glob(glob_path)
                println(file[end-14:end])
                rm(file)
            end
        elseif key == "r"
            print("Rewatching the trajectory.")
            MeshCatMechanisms.animate(mvis, ts, qs; realtimerate=2.)
        else 
            print("Key not recognized. Please use 'k', 't', or 'r'.")
        end
    end


end




