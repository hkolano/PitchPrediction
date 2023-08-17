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
    # @show ζ
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

function define_ee_task(this_state, des_σdot, dofs="all")
    task_dict = Dict{}()
    J = get_des_movement_jacobian(this_state)
    σdot_i = des_σdot
    task_dict["name"] = "ee"
    task_dict["type"] = 1 # equality task
    task_dict["dofs"] = dofs
    task_dict["is_static"] = false
    task_dict["update_func"] = update_ee_task!
    if dofs == "pos"
        task_dict["J"] = J[4:6,:]
        task_dict["σdot_i"] = des_σdot[4:6]
    elseif dofs == "ori"
        task_dict["J"] = J[1:3,:]
        task_dict["σdot_i"] = des_σdot[1:3]
    else
        task_dict["J"] = J
        task_dict["σdot_i"] = des_σdot
    end
    return task_dict
end

function update_ee_task!(task_dict::Dict, this_state, des_σdot)
    J = get_des_movement_jacobian(this_state)
    if task_dict["dofs"] == "pos"
        task_dict["J"] = J[4:6,:]
        task_dict["σdot_i"] = des_σdot[4:6]
    elseif task_dict["dofs"] == "ori"
        task_dict["J"] = J[1:3,:]
        task_dict["σdot_i"] = des_σdot[1:3]
    else
        task_dict["J"] = J
        task_dict["σdot_i"] = des_σdot
    end
end

function define_zero_pitch_task()
    task_dict = Dict{String, Any}()
    task_dict["name"] = "nopitch"
    task_dict["J"] = zeros(1, 11)
    task_dict["J"][2] = 1.
    task_dict["σdot_i"] = [0.]
    task_dict["type"] = 1 # equality task
    task_dict["is_static"] = true
    return task_dict
end

function define_zero_roll_task()
    task_dict = Dict{String, Any}()
    task_dict["name"] = "noroll"
    task_dict["J"] = zeros(1, 11)
    task_dict["J"][1] = 1.
    task_dict["σdot_i"] = [0.]
    task_dict["type"] = 1 # equality task
    task_dict["is_static"] = true
    return task_dict
end

function define_joint_limit_task(jointlab="shoulder", ϵ_safety=0.1, ϵ_activation=0.1)
    task_dict = Dict{String, Any}()
    task_dict["name"] = "jointlim_"*jointlab

    # Generate the Jacobian
    task_dict["J"] = zeros(1, 11)
    jt_idx = findfirst(isequal(jointlab), dof_names)
    task_dict["jt_idx"] = jt_idx
    task_dict["J"][jt_idx] = 1.

    # Define thresholds for this task
    task_dict["min"] = joint_lim_dict[jointlab][1]
    task_dict["max"] = joint_lim_dict[jointlab][2]
    task_dict["sl"] = task_dict["min"] + ϵ_safety # lower safety limit
    task_dict["al"] = task_dict["sl"] + ϵ_activation # lower activation limit
    task_dict["su"] = task_dict["max"] - ϵ_safety # upper safety limit 
    task_dict["au"] = task_dict["su"] - ϵ_activation # upper activation limit

    task_dict["type"] = 0 # set-based task
    task_dict["is_static"] = false

    # task_dict["is_active"] = true
    return task_dict
end

function update_joint_lim_task!(task, this_state)
    task_value = this_state[task["jt_idx"]]
    if task_value >= task_dict["au"]
        task_dict["σdot_i"] = task_dict["su"]
    elseif task_value <= task_dict["al"]
        task_dict["σdot_i"] = task_dict["sl"]
    else
        task_dict["σdot_i"] = []
        println("Task has not exceeded activation threshold values.")
    end
end

function define_zero_unactuated_task()
    task_dict = Dict{}()
    task_dict["name"] = "nopitchroll"
    task_dict["J"] = zeros(2, 11)
    task_dict["J"][1, 1] = 1.
    task_dict["J"][2, 2] = 1.
    task_dict["σdot_i"] = [0, 0.]
    task_dict["type"] = 1 # equality task
    task_dict["is_static"] = true
    return task_dict
end

"""
    tpik(task_list, this_state, des_σdot)

returns ζ for given task list.
No closed loop term
"""
function tpik(task_list, this_state, des_σdot)
    # Special consideration for first task
    task1 = task_list[1]
    if task1["name"] == "ee" && task1["is_static"] == false
        update_ee_task!(task1, this_state, des_σdot)
    elseif task1["is_static"] == false
        print("Need to update this task.")
    end
    J1_pinv = get_mp_pinv(task1["J"])
    ζ_old = J1_pinv*task1["σdot_i"]

    JA = task1["J"]
    N = I - get_mp_pinv(JA)*JA

    # Do the rest of the Tasks
    if length(task_list) > 1
        for task in task_list[2:end]
            # update jacobian if necessary
            if task["name"] == "ee" && task["is_static"] == false
                update_ee_task!(task, this_state, des_σdot)
            elseif task["is_static"] == false
                print("Need to update this task.")
            end

            # calculate ζ_n
            ζ_new = ζ_old + get_mp_pinv(task["J"]*N)*(task["σdot_i"] - task["J"]*ζ_old)
            
            # update augmented Jacobian and null space
            JA = [JA; task["J"]]
            N = I - get_mp_pinv(JA)*JA
            ζ_old = ζ_new
        end
    end
    return ζ_old
end

function get_all_task_combinations(task_list)
    is_equality_task = [task["type"] for task in task_list]
    num_set_based_tasks = count(x->x==0, is_equality_task)
    num_combos = 2^num_set_based_tasks
    seed_vec = Vector{Int64}(undef, num_set_based_tasks)
    possible_task_lists =[]
    for n = 1:num_combos
        this_task_combo = copy(digits!(seed_vec, n, base=2))

        this_task_mask = [Bool(x) for x in is_equality_task]
        for (index, value) in enumerate(this_task_mask)
            if value == false
                this_task_mask[index] = Bool(popfirst!(this_task_combo))
            end 
        end
        mod_task_list = task_list[this_task_mask]
        println("Adding possible hierarchy to list:")
        show_task_list(mod_task_list)
        push!(possible_task_lists, mod_task_list)
    end
    return possible_task_lists
end

function show_task_list(task_list)
    for task in task_list
        println(task["name"])
    end
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

"""as defined by Moe, 2016"""
function in_T_RC(xdot, x, xmin, xmax)
    if xmin < x < xmax
        return true 
    elseif x <= xmin && xdot >= 0
        return true 
    elseif x <= xmin && xdot < 0
        return false 
    elseif x >= xmax && xdot <= 0
        return true 
    else
        return false
    end
end

#%%

print_intermediate_states = false

# function simple_ik_iterator(state)
    # des_σdot = [1., 0., 0., 0., 0., 0.]
    # des_σdot = [0., 1., 0, 0., 0., 0.]
    des_σdot = [0., -.5, 0., 0., 0., 0.]
    simTime = 1. #2*pi
    viewRate=0.5
    Δt = 0.01

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

    # Define the task hierarchy
    task_list = [
        define_joint_limit_task("shoulder")
        define_joint_limit_task("elbow")
        define_ee_task(new_state, des_σdot)
        define_zero_pitch_task()
    ]

    for t in range(0, stop=simTime, step=Δt)
        if print_intermediate_states == true
            println("----- new state-----")
            println("Current State (ori, then pos/joints):")
            println(convert_to_rpy(new_state.q[1:4]))
            println(new_state.q[5:end])
        end
        # new_des_σdot = compensate_for_rotational_offset(new_state, des_σdot)

        # Do inverse kinematics
        ζ = iCAT_jacobians(new_state, des_σdot)
        # ζ = tpik(task_list, new_state, des_σdot)
        # println("Desired zeta:")
        # println(ζ)

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
