using CSV, DataFrames, StatsPlots, YAML
using MeshCat, MeshCatMechanisms, MechanismGeometries
using RigidBodyDynamics

import YAML 

include("../data-generation/UVMSsetup.jl")
include("../data-generation/ConfigFiles/MagicNumBlueROVHardware.jl")

urdf_file = joinpath("urdf", "blue_rov_hardware.urdf")
mech_blue_alpha, mvis, joint_dict, body_dict = mechanism_reference_setup(urdf_file)

# set_configuration!(mvis, joint_dict["base"], deg2rad(298-175.0))
include("../data-generation/TrajGenJoints.jl")

#%%

# Setup 
function new_plot()
    plot(xlabel = "Time (s)")
end 

# all_traj_codes = ["_alt_001-0", "_alt_001-1", "_alt_001-2", 
# "_alt_002-0", "_alt_002-1", 
# "_alt_008-0", 
# "_alt_008-1", "_alt_008-2", 
# "_alt_009-0", "_alt_009-1", 
# "_alt_011-0", "_alt_011-1", "_alt_011-2"]
all_traj_codes = ["003-0", "003-1", 
"004-0", "004-1", 
"005-0", "005-1", "005-2", "005-3", 
"006-0", "006-1", "006-2", "006-3", "006-4", 
"007-0", "007-1",
"009-0", "009-1",
"012-0", "012-1", "012-2", "012-3",
"014-0", "014-1", "014-2", 
"015-0", "015-1", 
"016-0", "016-1", 
"019-0", "019-1", "019-2", "019-3",
"020-0", "020-1", 
"024-0", "024-1", "024-2", 
"025-0", "025-1", 
"026-0", "026-1", "026-2", 
"030-0", "030-1"]

dataset = "fullrange2"
# dataset = "otherhome"

# Load Hinsdale arm traj data
trial_code = all_traj_codes[1]

js_filepath = joinpath("data", "hinsdale-data-2023", "traj"*trial_code*"_joint_states.csv")
js_df = CSV.read(js_filepath, DataFrame)
dropmissing!(js_df)

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
poses = Array{Float64}(undef, num_its, num_trajectory_dofs-1)
vels = Array{Float64}(undef, num_its, num_trajectory_dofs-1)
a = Array{Float64}(undef, num_trajectory_dofs-1, 6)

state = MechanismState(mech_blue_alpha)
zero!(state)
qs = typeof(configuration(state))[]

# reconstruct trajectory
for i in 1:num_trajectory_dofs-1
    dof_name = dof_names[i+6]
    a[i,:] = get_coeffs(pts, duration, i)
    (poses[:,i], vels[:,i]) = get_path!(poses[:,i], vels[:,i], pts.start.θs[i], pts.goal.θs[i], duration, a[i,:], num_its)
end

this_traj_params = quinticTrajParams(a, pts, duration)

for i in 1:num_its
    new_state_qs = copy(configuration(state))
    new_state_qs[8:11] = poses[i,1:4]
    set_configuration!(state, new_state_qs)
    push!(qs, copy(configuration(state)))
end

ts = collect(range(0, stop=duration, length=length(qs)))

#%%
offset = 5.3
des_df = DataFrame(time_secs=ts.+offset, b_des_poses=poses[:,4], c_des_poses=poses[:,3], d_des_poses=poses[:,2], e_des_poses=poses[:,1])

# PLOT 

# actual_palette = palette(cgrad(:greens), 4) 
desired_palette = palette([:deepskyblue2, :magenta], 4)
actual_palette = palette([:goldenrod1, :springgreen3], 4)

p_js = new_plot()
@df js_df plot!(p_js, :time_secs, cols(3:6); palette=actual_palette, linewidth=2)
xaxis!(p_js, grid = (:x, :solid, .75, 0.9), minorgrid = (:x, :dot, 0.5, .5))
@df des_df plot!(p_js, :time_secs, cols(2:5); palette=desired_palette, linewidth=2, linestyle=:dash)
ylabel!("Joint position (rad)")

#%%
data = Dict("start_time"=>offset)
metadata_file_loc = joinpath("data", "hinsdale-data-2023", "metadata_"*trial_code*".yml")
YAML.write_file(metadata_file_loc, data)