function jacobian_transpose_ik!(state::MechanismState,
                                traj::trajParams,
                                c::CtlrCache,
                                t,
                                α=0.001)
    mechanism = state.mechanism
    world = root_frame(mechanism)

    q = copy(configuration(state))

    ζ = get_zeta(traj, t, state, c)
    Δq = α*ζ
    q .=configuration(state) .+ Δq 
    set_configuration!(state, q)
end

function rotational_velocity_map(α, β, γ)
    Tθ = [  1       0           sin(β);
            0       cos(α)      -sin(α)*cos(β);
            0       sin(α)      cos(α)*cos(β)]
    TA = [I zeros(3,3) 
    zeros(3,3) Tθ]
end

# try https://www.youtube.com/watch?v=Z8nwjouP58o


function simplest_ik_transpose(this_state, des_σdot)
    # Get geometric Jacobian
    J = Array(calculate_full_ee_jacobian(this_state))
    # Get current pose
    current_pose = inv(relative_transform(this_state, root_frame(this_state.mechanism), default_frame(body_dict["jaw"])))
    R_I_B = rotation(current_pose)
    R_euler = Rotations.RotXYZ(R_I_B)
    # @show R_euler.theta1
    # @show R_euler.theta2 
    # @show R_euler.theta3
    TA = rotational_velocity_map(R_euler.theta1, R_euler.theta2, R_euler.theta3)
    # @show TA[1:3,1:3]
    JA = inv(TA)*J
    # JA = calc_jacobian_from_scratch(this_state)
    ζ = get_mp_pinv(J)*des_σdot
    @show ζ
    # ζ = get_mp_pinv(J)*des_σdot
    return ζ
end

function point_ik_transpose(state, des_σdot)
    point = Point3D(default_frame(jaw_body), 0., 0., 0.)
    Jp = point_jacobian(state, p_arm, transform(state, point, root_frame(mechanism)))
    ζ = get_mp_pinv(Array(Jp))*des_σdot[4:6]
    # println(ζ[11])
end

function skew(x1, x2, x3)
    output = [0 -x3 x2;
            x3 0 -x1;
            -x2 x1 0]
end

function calc_jacobian_from_scratch(state)
    # ---- Gather variables ----
    # Consider 'arm base' and 'vehicle frame' are coincident
    # Rotation of vehicle wrt world
    R_v_I_quat = configuration(state, joint_dict["vehicle"])[1:4]
    R_v_I_mat = QuatRotation(R_v_I_quat)
    # vector connecting the origin of the vehicle frame and the origin of the arm base frame, expressed in the vehicle frame
    # r_v0_v = translation(RigidBodyDynamics.frame_definitions(body_dict["vehicle"])[5])
    r_v0_v = [0., 0., 0.]
    R_0_I = R_v_I_mat
    # end effector position in the vehicle frame 
    η_ee_v = translation(relative_transform(state, default_frame(body_dict["jaw"]), default_frame(body_dict["vehicle"])))
    # arm jacobian 
    path_arm_only = RigidBodyDynamics.path(state.mechanism, body_dict["vehicle"], body_dict["jaw"])
    J_arm_full = geometric_jacobian(state, path_arm_only)
    J_pos_I = R_0_I*Array(J_arm_full)[4:6,7:end]
    J_ori_I = R_0_I*Array(J_arm_full)[1:3,7:end]

    J_pos_full = hcat(skew(R_v_I_mat*η_ee_v...)*R_v_I_mat, R_v_I_mat, J_pos_I)
    J_ori_full = hcat(R_v_I_mat, zeros(3,3), J_ori_I)
    J = vcat(J_ori_full, J_pos_full)
end

function compensate_for_rotational_offset(this_state, des_σdot)
    this_pose = inv(relative_transform(this_state, root_frame(state.mechanism), default_frame(body_dict["jaw"])))
    @show translation(this_pose)
    new_des_lin = cross(translation(this_pose),  des_σdot[1:3])
    new_des_lin = new_des_lin + des_σdot[4:6]
    return [des_σdot[1:3]; new_des_lin]
end

#%%
# function simple_ik_iterator(state)
    # des_σdot = [1., 0., 0., 0., 0., 0.]
    # des_σdot = [0., 1., 0, 0., 0., 0.]
    des_σdot = [0, 0, 1, 0, 0, .1]
    simTime = 1 #2*pi
    viewRate=0.5
    mechanism = state.mechanism
    qs = typeof(configuration(state))[]

    new_state = MechanismState(state.mechanism)
    reset_to_equilibrium!(new_state)
    # set_configuration!(new_state, joint_dict["vehicle"], [.704, .062, .062, .704, 0.1, 0, 0])
    new_state_qs = copy(configuration(new_state))
    prev_state_qs = copy(configuration(new_state))
   
    Δt = 0.05
    prev_ωb = [0., 0., 0.]
    prev_rot = Rotations.QuatRotation(new_state_qs[1:4])
    og_rpy = convert_to_rpy(new_state_qs[1:4])
    
    current_pose = inv(relative_transform(new_state, root_frame(mechanism), default_frame(body_dict["jaw"])))
    println("Current EE location:")
    println(translation(current_pose))

    og_ee_ori = RotMatrix(rotation(current_pose))
    # @show og_ee_ori
    og_ee_ori_taitB = RotXYZ(og_ee_ori)
    println("Current EE Orientation:")
    println(og_ee_ori_taitB.theta1, "  ", og_ee_ori_taitB.theta2, "  ", og_ee_ori_taitB.theta3)
    
    p_arm = get_ee_path(mechanism, body_dict["jaw"])
    body_frame = default_frame(body_dict["vehicle"])

    

    for t in range(0, stop=simTime, step=Δt)
        println("----- new state-----")
        println("Current State (ori, then pos/joints):")
        println(convert_to_rpy(new_state.q[1:4]))
        println(new_state.q[5:end])

        new_des_σdot = compensate_for_rotational_offset(new_state, des_σdot)

        # Do inverse kinematics
        ζ = simplest_ik_transpose(new_state, new_des_σdot)
        # ζ = point_ik_transpose(new_state, des_σdot)
        # println("Desired zeta:")
        # println(ζ)
        # @show ζ[1:3]
        # @show ζ[4:6]


        des_veh_twist_body = RigidBodyDynamics.Twist(body_frame, root_frame(mechanism), body_frame, SVector{3}(ζ[1:3]), SVector{3}(ζ[4:6])) 
        des_veh_twist_space = transform(des_veh_twist_body, transform_to_root(new_state, body_dict["vehicle"]))
        # println(linear(des_twist_space))
        
        # Integrate rotation
        new_R = prev_rot*exp(skew(ζ[1:3]...)*Δt)
        new_rot = Rotations.RotMatrix(SMatrix{3,3}(new_R))
        # @show new_rot
        new_rot = QuatRotation(new_rot)
        new_state_qs[1:4] .= Rotations.params(new_rot)

        # # Integrate velocities 
        # @show prev_rot*linear(des_veh_twist_body)
        new_state_qs[5:7] = prev_state_qs[5:7]+Δt*prev_rot*linear(des_veh_twist_body)
        # new_state_qs[5:7] = prev_state_qs[5:7]+Δt*ζ[4:6]

        # euler integrate joint poses 
        new_state_qs[8:12] = prev_state_qs[8:12] + Δt*ζ[7:11]
        # @show new_state_qs
        # @show convert_to_rpy(new_state_qs[1:4])
        set_configuration!(new_state, new_state_qs)
        push!(qs, copy(configuration(new_state)))

        prev_ωb = ζ[1:3]
        prev_rot = new_rot
        prev_state_qs = new_state_qs
    end

    final_pose = inv(relative_transform(new_state, root_frame(mechanism), default_frame(bodies(state.mechanism)[end])))
    println("Current EE location:")
    println(translation(final_pose))

    println("Distance travelled:")
    println(translation(final_pose)-translation(current_pose))

    println("Orientation Change:")
    final_ee_ori = RotMatrix(rotation(final_pose))
    final_ee_ori_taitB = RotXYZ(final_ee_ori)
    # @show og_ee_ori_taitB
    # @show final_ee_ori_taitB
    ori_diff = [final_ee_ori_taitB.theta1 - og_ee_ori_taitB.theta1, final_ee_ori_taitB.theta2 - og_ee_ori_taitB.theta2, final_ee_ori_taitB.theta3 - og_ee_ori_taitB.theta3]
    println(ori_diff)
    # println(convert_to_rpy(new_state_qs[1:4]) - og_rpy)

    ts = collect(range(0, stop=1, length=length(qs)))
    MeshCatMechanisms.animate(mvis, ts, qs; realtimerate=viewRate)
# end


# twist_body = RigidBodyDynamics.Twist(body_frame, base_frame, body_frame, SVector{3}(ζ[1:3]), SVector{3}(ζ[4:6])) 
# twist_space = transform(twist, transform_to_root(new_state, vehicle_body))
