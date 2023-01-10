using RigidBodyDynamics
using LinearAlgebra, StaticArrays, DataStructures
using MeshCat, MeshCatMechanisms, MechanismGeometries
using CoordinateTransformations
using GeometryBasics
using Printf, Plots, Revise

urdf_file = joinpath("urdf", "blue_rov.urdf")
mech_blue_alpha = parse_urdf(urdf_file; floating=true, gravity = [0.0, 0.0, 0.0])

mvis = MechanismVisualizer(mech_blue_alpha, URDFVisuals(urdf_file), vis[:alpha])
vehicle_joint, base_joint, shoulder_joint, elbow_joint, wrist_joint = joints(mech_blue_alpha)
~, vehicle_body, shoulder_body, upper_arm_body, elbow_body, wrist_body = bodies(mech_blue_alpha)

# moore-penrose pseudo-inverse
# noodle = [1; 2]
# noodle_mpp = pinv(noodle)

state = MechanismState(mech_blue_alpha)
zero!(state)
set_configuration!(state, vehicle_joint, [.993607, 0., 0.11289, 0.00034, 0., 0., 0.])
render(mvis)

# Define a point at the gripper
ee_point = Point3D(default_frame(wrist_body), 0., 0, -0.17)
world_frame = root_frame(mech_blue_alpha)
setelement!(mvis, ee_point, 0.01)

# What is the "path" variable in point_jacobian?
p = RigidBodyDynamics.path(mech_blue_alpha, root_body(mech_blue_alpha), wrist_body)

# Call point_jacobian
Jp = point_jacobian(state, p, transform(state, ee_point, world_frame))
Jp_array = Array(Jp)

T_v_I = relative_transform(state, default_frame(vehicle_body),  world_frame)
R_v_I = rotation(T_v_I)