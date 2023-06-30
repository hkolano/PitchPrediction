using MeshCat, MeshCatMechanisms, MechanismGeometries
using RigidBodyDynamics

import YAML

include("UVMSsetup.jl")
include("ConfigFiles/MagicNumInvKin.jl")

urdf_file = joinpath("urdf", "blue_rov_revjaw.urdf")
mech_blue_alpha, mvis, joint_dict, body_dict = mechanism_reference_setup(urdf_file)

#%%

include("TrajGenJoints.jl")

# get and parse yaml file
data = YAML.load_file("src/full-sim/data-generation/traj_yaml_files/traj0.yaml")
start_state = jointState(reverse(data["start_pos"]), reverse(data["start_vel"]))
end_state = jointState(reverse(data["end_pos"]), reverse(data["end_vel"]))
pts = Waypoints(start_state, end_state)
duration = data["duration"]

num_its=10

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
MeshCatMechanisms.animate(mvis, ts, qs; realtimerate=1.)




