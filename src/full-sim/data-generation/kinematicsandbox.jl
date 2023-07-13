"""
    rotational_velocity_map(α, β, γ)

Deprecated. Used to find TΘ used to transform between geometric and analytic jacobians. 
"""
function rotational_velocity_map(α, β, γ)
    Tθ = [  1       0           sin(β);
            0       cos(α)      -sin(α)*cos(β);
            0       sin(α)      cos(α)*cos(β)]
    TA = [I zeros(3,3) 
    zeros(3,3) Tθ]
end


"""
    funky_ik_transpose(this_state, des_σdot)

Deprecated. (hypothetically) Transforms the geometric Jacobian into an analytic Jacobian. 
Not confident this ever worked. 

# try https://www.youtube.com/watch?v=Z8nwjouP58o
"""
function funky_ik_transpose(this_state, des_σdot)
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

"""
    compensate_for_rotational_offset(this_state, des_σdot)

Deprecated. Multiplies desired velocity vector by r_ee x ω to change the center of rotation 
from the world frame to the end effector frame. 
"""
function compensate_for_rotational_offset(this_state, des_σdot)
    this_pose = inv(relative_transform(this_state, root_frame(state.mechanism), default_frame(body_dict["jaw"])))
    # @show translation(this_pose)
    new_des_lin = cross(translation(this_pose),  des_σdot[1:3])
    new_des_lin = new_des_lin + des_σdot[4:6]
    return [des_σdot[1:3]; new_des_lin]
end

"""
    skew(x1, x2, x3)

Ouputs the skew-symmetric matrix representation of the three inputs. 
"""
function skew(x1, x2, x3)
    output = [0 -x3 x2;
            x3 0 -x1;
            -x2 x1 0]
end

function get_des_movement_jacobian(this_state)
    # Get geometric Jacobian
    J = Array(calculate_full_ee_jacobian(this_state))

    r_ee = translation(inv(relative_transform(this_state, root_frame(this_state.mechanism), default_frame(body_dict["jaw"]))))
    J_mod = inv([I zeros(3,3); skew(r_ee...) I])*J
    return J_mod
end

function compose_jacobians(this_state, des_σdot)
    J_i[1] = [Matrix{Int}(I, 6, 6) zeros(6, 5)]

    σdot_veh = velocity(this_state)[1:6]
    A = diagm(vec([1 1 0 0 0 0]))
    J_i_A[1] = A*J_i[1]
    ζ_i[1] = get_mp_pinv(A*J_i[1])*A*σdot_veh
    # ζ_i[1] = get_mp_pinv(J_i[1])*(diagm(vec([1 1 0 0 0 0]))*σdot_veh)
 
    J_i[2] = get_des_movement_jacobian(this_state)
    J_i_A[2] = vcat(J_i_A[1], J_i[2])
    N_i_A[1] = I - get_mp_pinv(J_i_A[1])*J_i_A[1]
    # @show N_i_A[1]

    ζ_i[2] = ζ_i[1] + get_mp_pinv(J_i[2]*N_i_A[1])*(des_σdot - J_i[2]*ζ_i[1])
    # @show ζ_i[2]'
    # J_movement = get_des_movement_jacobian(this_state)
    # ζ = get_mp_pinv(J_movement)*des_σdot
    return ζ_i[2]
end

function tpik(this_state, des_σdot)
    # Task σ1: end effector following
    J1 = get_des_movement_jacobian(this_state)
    des_ζ = get_mp_pinv(J1)*des_σdot
end

function iCAT_jacobians(this_state, des_σdot)
    ρ0 = zeros(11)
    Q0 = I

    # Set up first task
    J1 = [Matrix{Int}(I, 6, 6) zeros(6, 5)]
    σdot_veh = velocity(this_state)[1:6]
    # σdot_veh = velocity(state)[1:6]
    A1 = diagm(vec([1 1 0 0 0 0]))

    W1 = J1*Q0*regd_inverse(J1*Q0, A1, Q0)
    Q1 = Q0*(I-regd_inverse(J1*Q0, A1, I)*J1*Q0)
    T1 = I-Q0*regd_inverse(J1*Q0, A1, I)*W1*J1
    ρ1 = T1*ρ0 + Q0*regd_inverse(J1*Q0, A1, I)*W1*σdot_veh 

    J2 = get_des_movement_jacobian(this_state)
    # J2 = get_des_movement_jacobian(state)
    A2 = I 

    W2 = J2*Q1*regd_inverse(J2*Q1, A2, Q1)
    Q2 = Q1*(I-regd_inverse(J2*Q1, A2, I)*J2*Q1)
    T2 = I-Q1*regd_inverse(J2*Q1, A2, I)*W2*J2
    ρ2 = T2*ρ1 + Q1*regd_inverse(J2*Q1, A2, I)*W2*des_σdot 
end

function regd_inverse(X, A, Q)
    η = 1
    P = .1I
    B = X'*A*X + η*(I-Q)'*(I-Q)
    F = svd(B)
    # @show F.Vt
    inv(B + F.Vt*P*F.V)*X'*A*A
end

#%%
# function simple_ik_iterator(state)
    # des_σdot = [1., 0., 0., 0., 0., 0.]
    # des_σdot = [0., 1., 0, 0., 0., 0.]
    des_σdot = [0., 1., 0., 0., 0, 0.]
    simTime = 1 #2*pi
    viewRate=0.5
    Δt = 0.05

    mechanism = state.mechanism
    qs = typeof(configuration(state))[]

    new_state = MechanismState(state.mechanism)
    reset_to_equilibrium!(new_state)
    # set_configuration!(new_state, joint_dict["vehicle"], [.704, .062, .062, .704, 0.1, 0, 0])
    new_state_qs = copy(configuration(new_state))
    prev_state_qs = copy(configuration(new_state))
   
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

    # Null Space Projection containers
    N_i_A = Dict{Int, Any}(0 => I)
    J_i = Dict{Int, Array}()
    ζ_i = Dict{Int, Array}()
    J_i_A = Dict{Int, Array}()

    for t in range(0, stop=simTime, step=Δt)
        println("----- new state-----")
        println("Current State (ori, then pos/joints):")
        println(convert_to_rpy(new_state.q[1:4]))
        println(new_state.q[5:end])

        # new_des_σdot = compensate_for_rotational_offset(new_state, des_σdot)

        # Do inverse kinematics
        # ζ = iCAT_jacobians(new_state, des_σdot)
        ζ = tpik(new_state, des_σdot)
        println("Desired zeta:")
        println(ζ)

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
