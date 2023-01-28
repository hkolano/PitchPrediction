# ----------------------------------------------------------
#                         Definitions
# ----------------------------------------------------------
default_act_dof_idxs = 3:10

function get_ee_path(mech::Mechanism, ee_body)
    world_body=bodies(mech)[1]
    p_arm = RigidBodyDynamics.path(mech, world_body, ee_body)
end

function calculate_full_ee_jacobian(state::MechanismState)
    return geometric_jacobian(state, p_arm)
end

function calculate_actuated_jacobian(state::MechanismState)
    return Array(calculate_full_ee_jacobian(state))[:,1:10] # ignore the prismatic joint
end

function calc_pos_jacobian(state::MechanismState)
    return calculate_actuated_jacobian(state)[4:6,act_dof_idxs]
end

function calc_ori_jacobian(state::MechanismState)
    return calculate_actuated_jacobian(state)[1:3, act_dof_idxs]
end

# Moore-Penrose Pseudo-Inverse
function get_mp_pinv(J_mat)
    return pinv(J_mat)
end

function get_ee_task_error(traj::trajParams, t, state::MechanismState)
    des_pose, des_vel = get_des_state_at_t(t, traj)
    R_I_D = rotation(des_pose)
    current_pose = inv(relative_transform(state, base_frame, default_frame(bodies(state.mechanism)[end])))
    R_I_B = rotation(current_pose)
    
    R_error = transpose(R_I_D)*R_I_B 
    R_error_euler = RotXYZ(R_error)
    R_error_rpy = [R_error_euler.theta1, R_error_euler.theta2, R_error_euler.theta3]
    
    trans_error = translation(des_pose) - translation(current_pose)
    task_error = vcat(R_error_rpy, trans_error)
end
