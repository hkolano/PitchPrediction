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
    
    # rotation error = difference in rotation matrices
    R_error = transpose(R_I_D)*R_I_B 
    R_error_euler = RotXYZ(R_error)
    R_error_rpy = [R_error_euler.theta1, R_error_euler.theta2, R_error_euler.theta3]
    
    # alternative rotation error = difference in quaternion
    ee = QuatRotation(R_I_B)
    d = QuatRotation(R_I_D)
    ϵ_d = [d.x, d.y, d.z]
    ϵ_ee = [ee.x, ee.y, ee.z]
    R_error_quat = ee.w*ϵ_d - d.w*ϵ_ee + cross(ϵ_d, ϵ_ee)

    # translation error and packaging
    trans_error = translation(des_pose) - translation(current_pose)
    task_error = vcat(R_error_quat, trans_error)
end

function get_ee_task_error_and_ff_vel(traj::trajParams, t, state::MechanismState)
    des_pose, des_vel = get_des_state_at_t(t, traj)
    R_I_D = rotation(des_pose)
    current_pose = inv(relative_transform(state, base_frame, default_frame(bodies(state.mechanism)[end])))
    R_I_B = rotation(current_pose)
    
    R_error = transpose(R_I_D)*R_I_B 
    R_error_euler = RotXYZ(R_error)
    R_error_rpy = [R_error_euler.theta1, R_error_euler.theta2, R_error_euler.theta3]

    # alternative rotation error = difference in quaternion
    ee = QuatRotation(R_I_B)
    d = QuatRotation(R_I_D)
    ϵ_d = [d.x, d.y, d.z]
    ϵ_ee = [ee.x, ee.y, ee.z]
    R_error_quat = ee.w*ϵ_d - d.w*ϵ_ee + cross(ϵ_d, ϵ_ee)
    
    trans_error = translation(des_pose) - translation(current_pose)
    task_error = vcat(R_error_quat, trans_error)

    vel_rot = rotation(des_vel)
    vel_rot_euler = RotXYZ(R_error)
    vel_rot_rpy = [vel_rot_euler.theta1, vel_rot_euler.theta2, vel_rot_euler.theta3]
    vel_trans = translation(des_vel)
    des_vel_vec = vcat(vel_rot_rpy, vel_trans)
    return task_error, des_vel_vec
end
