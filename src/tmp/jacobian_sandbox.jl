using RigidBodyDynamics
using LinearAlgebra, StaticArrays, DataStructures
using MeshCat, MeshCatMechanisms, MechanismGeometries
using CoordinateTransformations
using GeometryBasics
using Printf, Plots, Revise


#%%

urdf_file = joinpath("urdf", "blue_rov.urdf")
mech_blue_alpha = parse_urdf(urdf_file; floating=true, gravity = [0.0, 0.0, 0.0])
vehicle_joint, base_joint, shoulder_joint, elbow_joint, wrist_joint, jaw_joint = joints(mech_blue_alpha)
world_body, vehicle_body, shoulder_body, upper_arm_body, elbow_body, wrist_body, jaw_body = bodies(mech_blue_alpha)

mvis = MechanismVisualizer(mech_blue_alpha, URDFVisuals(urdf_file))

state = MechanismState(mech_blue_alpha)
zero!(state)
# set_configuration!(state, vehicle_joint, [.993607, 0., 0.11289, 0.00034, 1., 0., 0.])
set_configuration!(state, vehicle_joint, [1., 0., 0., 0., 0., 0., 0.])

#%%
p_arm = RigidBodyDynamics.path(mech_blue_alpha, world_body, jaw_body)
act_dof_idxs = 3:10

function calculate_full_ee_jacobian(state)
    return geometric_jacobian(state, p_arm)
end

function calculate_actuated_jacobian(state)
    return Array(calculate_full_ee_jacobian(state))
end

function calc_pos_jacobian(state)
    return calculate_actuated_jacobian(state)[4:6,act_dof_idxs]
end

function calc_ori_jacobian(state)
    return calculate_actuated_jacobian(state)[1:3, act_dof_idxs]
end

# Moore-Penrose Pseudo-Inverse
function get_mp_pinv(J_mat)
    return pinv(J_mat)
end

floop = calculate_actuated_jacobian(state)
Jt = get_mp_pinv(floop)




