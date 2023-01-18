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

# moore-penrose pseudo-inverse
# noodle = [1; 2]
# noodle_mpp = pinv(noodle)

#%%
p_arm = RigidBodyDynamics.path(mech_blue_alpha, world_body, jaw_body)
J_arm_from_veh_in_world = geometric_jacobian(state, p_arm)

J_arm_from_veh_mat = Array(J_arm_from_veh_in_world)

# J_pos_0 = J_arm_from_armbase_mat[4:6, 7:10]
# J_ori_0 = J_arm_from_armbase_mat[1:3, 7:10]



