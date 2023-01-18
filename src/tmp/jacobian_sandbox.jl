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

axis = SVector(0., 1., 0.) # joint axis
I_1 = 0.0 # moment of inertia about joint axis
c_1 = 0.0 # center of mass location with respect to joint axis
m_1 = 0.0 # mass
frame1 = CartesianFrame3D("ee_link") # the reference frame in which the spatial inertia will be expressed
inertia1 = SpatialInertia(frame1,
    moment=I_1 * axis * axis',
    com=SVector(0, 0, c_1),
    mass=m_1)

ee_link = RigidBody(inertia1)
ee_joint = Joint("ee_joint", Revolute(axis))
before_ee_to_after_wrist = Transform3D(frame_before(ee_joint), frame_after(wrist_joint), SVector(0, 0, -.19))
attach!(mech_blue_alpha, wrist_body, ee_link, ee_joint, joint_pose=before_ee_to_after_wrist)

#%%

mvis = MechanismVisualizer(mech_blue_alpha, URDFVisuals(urdf_file))
setelement!(mvis, default_frame(jaw_body))
state = MechanismState(mech_blue_alpha)
zero!(state)
set_configuration!(state, vehicle_joint, [.993607, 0., 0.11289, 0.00034, 1., 0., 0.])
# set_configuration!(state, vehicle_joint, [1., 0., 0., 0., 0., 0., 0.])

setelement!(mvis, default_frame(ee_link))
# moore-penrose pseudo-inverse
# noodle = [1; 2]
# noodle_mpp = pinv(noodle)

# Define a point at the gripper
ee_point = Point3D(default_frame(wrist_body), 0., 0, -0.19)
world_frame = root_frame(mech_blue_alpha)
setelement!(mvis, ee_point, 0.01)

ee_point_ref_itself = Point3D(default_frame(ee_link), 0., 0., 0.)

#%%
# What is the "path" variable in point_jacobian?
p = RigidBodyDynamics.path(mech_blue_alpha, root_body(mech_blue_alpha), ee_link)

# Call point_jacobian
Jp = point_jacobian(state, p, transform(state, ee_point_ref_itself, world_frame))
Jp_array = Array(Jp)

# T_v_I = relative_transform(state, default_frame(vehicle_body),  world_frame)
# R_v_I = rotation(T_v_I)

jaw_transf = RigidBodyDynamics.frame_definitions(wrist_body)[4]
jaw_frame = jaw_transf.from 
# RigidBodyDynamics.change_default_frame!(wrist_body, inv(jaw_transf).from)
# setelement!(mvis, default_frame(wrist_body), 0.3)

# Arm base frame
F_0 = RigidBodyDynamics.frame_definitions(vehicle_body)[5].from
# from v to 0, but in 0 coords
T_v_0_in0 = fixed_transform(vehicle_body, default_frame(vehicle_body), F_0)
setelement!(mvis, F_0, 0.3)

# from 0 to I, in I coords
T_0_I = relative_transform(state, F_0, world_frame)
# R_0_I = rotation(T_0_I)

# T_I_0 = relative_transform(state, world_frame, F_0)

# # # O_3_3 = zeros(3, 3)
# # J_ori_partial = hcat(R_v_I, O_3_3)

p_arm = RigidBodyDynamics.path(mech_blue_alpha, world_body, ee_link)
J_arm_from_veh_in_world = geometric_jacobian(state, p_arm)

J_arm_from_veh_mat = Array(J_arm_from_veh_in_world)

# # # I think this is the jacobian of the 
# J_arm_from_armbase = transform(J_arm_from_veh_in_world, T_I_0)
# J_arm_from_armbase_mat = Array(J_arm_from_armbase)

# J_pos_0 = J_arm_from_armbase_mat[4:6, 7:10]
# J_ori_0 = J_arm_from_armbase_mat[1:3, 7:10]



