using Rotations

function convert_to_rpy(quat_vals)
    quat_rot = QuatRotation(quat_vals...)
    euler = RotXYZ(quat_rot)
    vals = [euler.theta1, euler.theta2, euler.theta3] 
end

function foo(urdf_file)
    vis = Visualizer()
    mech_blue_alpha = parse_urdf(urdf_file; floating=true, gravity = [0.0, 0.0, 0.0])
    delete!(vis)

    # Create visuals of the URDFs
    mvis = MechanismVisualizer(mech_blue_alpha, URDFVisuals(urdf_file), vis[:alpha])

    # Name the joints and bodies of the mechanism
    joint_dict = Dict{String, RigidBodyDynamics.Joint}()
    body_dict = Dict{String, RigidBodyDynamics.RigidBody}()
    frame_dict = Dict{String, CartesianFrame3D}()
    for (idx, link_name) in enumerate(body_names)
        joint_dict[link_name] = joints(mech_blue_alpha)[idx]
        body_dict[link_name] = bodies(mech_blue_alpha)[idx+1]
        frame_dict[link_name] = default_frame(body_dict[link_name])
    end
    return mech_blue_alpha, mvis, joint_dict, body_dict, frame_dict
end