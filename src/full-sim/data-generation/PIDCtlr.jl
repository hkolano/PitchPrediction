using RigidBodyDynamics, Distributions, Random

# ------------------------------------------------------------------------
#                              SETUP 
# ------------------------------------------------------------------------
arm_Kp = .024
arm_Kd = 0.0032
arm_Ki = 0.00001
v_Kp = .18 
v_Kd = 0.0001 
v_Ki = 0.012
Kp = [50, v_Kp, v_Kp, v_Kp, arm_Kp, arm_Kp, arm_Kp, .0004, 20.]
Kd = [.5, v_Kd, v_Kd, v_Kd, arm_Kd, arm_Kd, arm_Kd, 0.00004, .002]
Ki = [1.5, v_Ki, v_Ki, v_Ki, arm_Ki, arm_Ki, arm_Ki, 0.0008, .1]
torque_lims = [20., 71.5, 88.2, 177., 10.0, 10.0, 10.0, 0.6, 600]

# Sensor noise distributions 
# Implemented:
# Encoder --> joint position noise -integration-> joint velocity noise
# Gyroscope --> vehicle body vel noise 
v_ang_vel_noise_dist = Distributions.Normal(0, .0013) # 75 mdps (LSM6DSOX)
arm_pos_noise_dist = Distributions.Normal(0, .0017/6) # .1 degrees, from Reach website

v_lin_vel_noise_dist = Distributions.Normal(0, .01)
v_ori_noise_dist = Distributions.Normal(0, 0.01)
v_pos_noise_dist = Distributions.Normal(0, 0.01)

# v_ang_vel_noise_dist = Distributions.Normal(0, .00) # 75 mdps (LSM6DSOX)
# arm_pos_noise_dist = Distributions.Normal(0, .00) # .1 degrees, from Reach website

# v_lin_vel_noise_dist = Distributions.Normal(0, 0.)
# v_ori_noise_dist = Distributions.Normal(0, 0.0)
# v_pos_noise_dist = Distributions.Normal(0, 0.0)


mutable struct CtlrCache
    time_step::Float64
    ctrl_freq::Float64
    ctrl_steps::Float64
    vel_error_cache::Array{Float64}
    vel_int_error_cache::Array{Float64}
    step_ctr::Int
    joint_vec
    des_vel::Array{Float64}
    taus
    noisy_qs
    noisy_vs
    filtered_vs
    
    function CtlrCache(dt, control_frequency, mechanism)
        if length(joints(mechanism)) == 5
            vehicle_joint, jointE, jointD, jointC, jointB = joints(mechanism)
            joint_vec = [vehicle_joint, jointE, jointD, jointC, jointB]
            num_actuated_dofs = 8
            num_dofs = 10
        elseif length(joints(mechanism)) == 6
            vehicle_joint, jointE, jointD, jointC, jointB, jawjoint = joints(mechanism)
            joint_vec = [vehicle_joint, jointE, jointD, jointC, jointB, jawjoint]
            num_actuated_dofs = 9
            num_dofs = 11
        end
        ctrl_loop_num_steps = 4*(1/dt)/control_frequency
        new(dt, control_frequency, ctrl_loop_num_steps, #=
        =# zeros(num_actuated_dofs), zeros(num_actuated_dofs), 0, joint_vec, #=
        =# zeros(num_actuated_dofs), Array{Float64}(undef, num_dofs, 1), #=
        =# Array{Float64}(undef, num_dofs+1, 1), Array{Float64}(undef, num_dofs, 1), #=
        =# Array{Float64}(undef, num_dofs, 1)) 
    end
end

# ------------------------------------------------------------------------
#                          UTILITY FUNCTIONS
# ------------------------------------------------------------------------
"""
Given a `torque`` value and a torque `limit``, bounds `torque`` to be within the bounds of `limit`.

Assumes the torque limit is the same in the positive and negative directions.
"""
function impose_torque_limit!(torque, limit)
    if torque > limit
        torque = limit
    elseif torque < -limit
        torque = -limit
    end
end

function resettimestep(c::CtlrCache)
    c.step_ctr = 0
end

# ------------------------------------------------------------------------
#                              CONTROLLER
# ------------------------------------------------------------------------
"""
Imposes a PID controller to follow a velocity specified by TrajGen.get_desv_at_t().
In this case, also imposes damping to each joint. 
Requires the parameters of the trajectory to be followed (pars=trajParams), which consists of the quintic coefficients `a` and the two waypoints to travel between.
Only happens every 4 steps because integration is done with Runge-Kutta.
"""
# function pid_control!(torques::AbstractVector, t, state::MechanismState, pars, c)
function pid_control!(torques::AbstractVector, t, state::MechanismState, pars, c)
    # If it's the first time called in the Runge-Kutta, update the control torque
    # println("Made it inside the function! Ctr = $(time_step_ctr)")
    if rem(c.step_ctr, 4) == 0
        # println("Joint E Torques: $(torques[7])")
        # Set up empty vector for control torques
        c_taus = zeros(size(c.taus, 1),1)
        if c.step_ctr == 0
            torques[6] = 5.2 # ff z value
            torques[3] = 0.
            torques[5] = 0.
            torques[4] = -2.3 # ff x value
            torques[7] = .00 # ff joint E value
            torques[8] = -.325 # ff Joint D value 
            torques[9] = -.034 # ff Joint C value
            torques[10] = .004
        end
        
        # Roll and pitch are not controlled
        torques[1] = 0.
        torques[2] = 0.
        
        # println("Requesting des vel from trajgen")
        # c.des_vel = TrajGen.get_desv_at_t(t, pars)
        c.des_vel = get_desv_at_t(t, pars)
        if rem(c.step_ctr, 1000) == 0
            println("Desired velocity vector: $(c.des_vel)")
        end
        # Don't move the manipulator
        # c.des_vel[end-3:end] = zeros(4,1)

        if rem(c.step_ctr, c.ctrl_steps) == 0 && c.step_ctr != 0

            noisy_poses = similar(configuration(state))
            noisy_vels = similar(velocity(state))

            add_arm_noise!(noisy_poses, noisy_vels, configuration(state)[8:end], c.noisy_qs[8:end,end], 1/c.ctrl_freq)
            add_rotational_noise!(noisy_poses, noisy_vels, velocity(state)[1:3], c.noisy_qs[1:4,end], c.time_step)

            noisy_velocity_veh = add_velocity_noise(velocity(state))
            noisy_poses_veh = add_position_noise(configuration(state))

            noisy_poses[1:7] = noisy_poses_veh[1:7]
            noisy_vels[4:6] = noisy_velocity_veh[4:6]

            c.noisy_vs = cat(c.noisy_vs, noisy_vels, dims=2)
            c.noisy_qs = cat(c.noisy_qs, noisy_poses, dims=2)

            filtered_velocity = moving_average_filter_velocity(5, c.noisy_vs)
            c.filtered_vs = cat(c.filtered_vs, filtered_velocity, dims=2)

            # Get forces for vehicle (yaw, surge, sway, heave)
            for dir_idx = 3:6
                # println("PID ctlr on vehicle")
                actual_vel = velocity(state, c.joint_vec[1])
                ctlr_tau = PID_ctlr(torques[dir_idx][1], t, filtered_velocity[dir_idx], dir_idx, c)
                c_taus[dir_idx] = ctlr_tau 
                torques[dir_idx] = ctlr_tau
            end
            
            # Get torques for the arm joints
            for jt_idx in 2:length(c.joint_vec) # Joint index (1:vehicle, 2:baseJoint, etc)
                idx = jt_idx+5 # velocity index (7 to 10)
                ctlr_tau = PID_ctlr(torques[idx][1], t, filtered_velocity[idx], idx, c) 
                torques[velocity_range(state, c.joint_vec[jt_idx])] .= [ctlr_tau] 
                c_taus[idx] = ctlr_tau 
            end
            #TODO switch to push! ?
            c.taus = cat(c.taus, c_taus, dims=2)
            # push!(c.taus, copy(c_taus))
        end
    end
    if rem(c.step_ctr, 4000) == 0
        println("At time $(c.step_ctr/4000)...")
    end
    c.step_ctr = c.step_ctr + 1
end;

"""
Imposes a PID controller on one joint. 

`torque` = storage variable, does not modify
`t` = current time. Not in use. 
`vel_act` = the actual instantaneous velocity of the joint 
`des_vel` = the desired velocity of the joint, as found by the trajectory generator
`j_idx` = the index of the joint (of the actuated ones). Here, will be 1 or 2.

Returns a controller torque value, bounded by the torque limits and some dτ/dt value. 
"""
function PID_ctlr(torque, t, vel_act, idx, c)
    actuated_idx = idx-2
    d_vel = c.des_vel[actuated_idx]
    vel_error = vel_act[1] - d_vel
    d_vel_error = (vel_error - c.vel_error_cache[actuated_idx])/(1/c.ctrl_freq)
    # println("D_Velocity error on idx$(j_idx): $(d_vel_error)")
    c.vel_int_error_cache[actuated_idx] = c.vel_int_error_cache[actuated_idx] + vel_error*(1/c.ctrl_freq)
    d_tau = -Kp[actuated_idx]*vel_error - Kd[actuated_idx]*d_vel_error - Ki[actuated_idx]*c.vel_int_error_cache[actuated_idx]
    # println("Ideal tau: $(d_tau)")

    # Can only change torque a small amount per time step 
    # arm joints can change faster than thrusters
    5 <= actuated_idx ? lim = 0.0001 : lim = 0.001
    lim = lim*(1/c.time_step)/c.ctrl_freq
    d_tau = limit_d_tau(d_tau, lim)
    
    # Torque limits
    new_tau = torque .+ d_tau
    impose_torque_limit!(new_tau, torque_lims[actuated_idx])

    # # store velocity error term
    c.vel_error_cache[actuated_idx]=vel_error
    # c.vel_int_error = c.vel_int_error + vel_error

    return new_tau
end

function limit_d_tau(d_tau, limit)
    if d_tau < -limit
        d_tau = -limit
    elseif d_tau > limit
        d_tau = limit
    end
    return d_tau
end

function add_velocity_noise(velocity)
    noisy_velocity = similar(velocity)
    noisy_velocity[1:3] = velocity[1:3] + rand(v_ang_vel_noise_dist, 3)
    noisy_velocity[4:6] = velocity[4:6] + rand(v_lin_vel_noise_dist, 3)
    return noisy_velocity
end

function add_position_noise(configuration)
    noisy_poses = similar(configuration)
    noisy_poses[1:4] = configuration[1:4] + rand(v_ori_noise_dist, 4)
    noisy_poses[5:7] = configuration[5:7] + rand(v_pos_noise_dist, 3)
    return noisy_poses
end

function add_arm_noise!(noisy_poses, noisy_vels, joint_poses, last_noisy_joint_pose, dt)
    noisy_joint_poses = joint_poses + rand(arm_pos_noise_dist, length(joint_poses))
    noisy_velocity = (noisy_joint_poses - last_noisy_joint_pose)./dt
    noisy_poses[8:end] = noisy_joint_poses
    noisy_vels[7:end] = noisy_velocity
end

function add_rotational_noise!(noisy_poses, noisy_vels, body_vels, last_ori, dt)
    noisy_vels[1:3] = body_vels[1:3] + rand(v_ang_vel_noise_dist, 3)
end

function moving_average_filter_velocity(filt_size, noisy_vs)
    num_its = size(noisy_vs, 2)
    if num_its < filt_size
        filtered_vels = sum(noisy_vs[:,end-num_its+1:end], dims=2) ./ num_its
    else
        filtered_vels = sum(noisy_vs[:,end-filt_size+1:end], dims=2) ./ filt_size  
    end
    return filtered_vels
end
