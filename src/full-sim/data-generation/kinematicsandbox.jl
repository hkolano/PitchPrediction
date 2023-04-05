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
    # Tθ = [  0    cos(θy)      sin(θp)*sin(θy);
    #         1    0           cos(θp);
    #         0    sin(θy)      -sin(θp)*cos(θy)]
    # Tθ = [-sin(θp)*cos(θy)  sin(θy)     0;
    #     cos(θp)             0           1;
    #     sin(θp)*sin(θy)     cos(θy)     0]
    #TODO figure out how to make this right
    Tθ = [  1       0           sin(β);
            0       cos(α)      -sin(α)*cos(β);
            0       sin(α)      cos(α)*cos(β)]
    TA = [Tθ zeros(3,3) 
    zeros(3,3) I]
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
    ζ = get_mp_pinv(JA)*des_σdot
    # println(ζ)
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

#%%
# function simple_ik_iterator(state)
    # des_σdot = [1., 0., 0., 0., 0., 0.]
    # des_σdot = [0., 1., 0, 0., 0., 0.]
    des_σdot = [0., 0., 1., 0, 0., 0.]
    mechanism = state.mechanism
    qs = typeof(configuration(state))[]

    new_state = MechanismState(state.mechanism)
    reset_to_equilibrium!(new_state)
    # set_configuration!(new_state, joint_dict["vehicle"], [1, 0, 0, 0., 0, 0, 0])
    new_state_qs = copy(configuration(new_state))
    prev_state_qs = copy(configuration(new_state))
   
    Δt = 0.01
    prev_ωb = [0., 0., 0.]
    prev_rot = Rotations.QuatRotation(new_state_qs[1:4])
    og_rpy = convert_to_rpy(new_state_qs[1:4])
    
    current_pose = inv(relative_transform(new_state, root_frame(mechanism), default_frame(bodies(state.mechanism)[end])))
    println("Current EE location:")
    println(translation(current_pose))

    og_ee_ori = RotMatrix(rotation(current_pose))
    @show og_ee_ori
    og_ee_ori_eul = RotXYZ(og_ee_ori)
    println(og_ee_ori_eul.theta1, "  ", og_ee_ori_eul.theta2, "  ", og_ee_ori_eul.theta3)
    

    p_arm = get_ee_path(mechanism, body_dict["jaw"])
    body_frame = default_frame(body_dict["vehicle"])

    for t in range(0, stop=1, step=Δt)
        # println("----- new state-----")
        # println("Current Config:")
        # println(new_state.q)

        # Do inverse kinematics
        ζ = simplest_ik_transpose(new_state, des_σdot)
        # ζ = point_ik_transpose(new_state, des_σdot)
        # println("Desired zeta:")
        # println(ζ)
        # @show ζ[1:3]
        # @show ζ[4:6]


        des_veh_twist_body = RigidBodyDynamics.Twist(body_frame, root_frame(mechanism), body_frame, SVector{3}(ζ[1:3]), SVector{3}(ζ[4:6])) 
        des_veh_twist_space = transform(des_veh_twist_body, transform_to_root(new_state, body_dict["vehicle"]))
        # println(linear(des_twist_space))
        
        # Integrate rotation
        # Ω = (skew(ζ[1:3]...) - skew(prev_ωb...))/2
        # new_rot_mat = prev_rot*(I+(Δt/2)*Ω)*inv(I-(Δt/2)*Ω) # MP-R method from https://repozitorij.uni-lj.si/Dokument.php?id=85693&dn=
        new_R = prev_rot*exp(skew(ζ[1:3]...)*Δt)
        # Ṙ = Rotations.kinematics(prev_rot, ζ[1:3])
        # @show Ṙ
        # @show Ṙ*Δt*Rotations.params(QuatRotation(prev_rot))
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
    final_ee_ori_eul = RotXYZ(final_ee_ori)
    @show og_ee_ori_eul
    @show final_ee_ori_eul
    ori_diff = [final_ee_ori_eul.theta1 - og_ee_ori_eul.theta1, final_ee_ori_eul.theta2 - og_ee_ori_eul.theta2, final_ee_ori_eul.theta3 - og_ee_ori_eul.theta3]
    println(ori_diff)
    # println(convert_to_rpy(new_state_qs[1:4]) - og_rpy)

    ts = collect(range(0, stop=1, length=length(qs)))
    MeshCatMechanisms.animate(mvis, ts, qs; realtimerate=1.)
# end


# twist_body = RigidBodyDynamics.Twist(body_frame, base_frame, body_frame, SVector{3}(ζ[1:3]), SVector{3}(ζ[4:6])) 
# twist_space = transform(twist, transform_to_root(new_state, vehicle_body))
