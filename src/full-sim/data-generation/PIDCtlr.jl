using RigidBodyDynamics, Distributions, Random

# ------------------------------------------------------------------------
#                              SETUP 
# ------------------------------------------------------------------------
mutable struct CtlrCache
    n_cache 
    f_cache
    vel_error_cache::Array{Float64}
    vel_int_error_cache::Array{Float64}
    step_ctr::Int
    des_vel::Array{Float64}
    taus
    last_v̇
    traj_num
    
    function CtlrCache(state, n_cache, f_cache)
        mechanism = state.mechanism
        num_dofs = num_velocities(mechanism)
        num_actuated_dofs = num_dofs-2

        new(n_cache, f_cache, 
            zeros(num_actuated_dofs), zeros(num_actuated_dofs), 0, 
            zeros(num_actuated_dofs), Array{Float64}(undef, num_dofs, 1), 
            zeros(num_dofs), 1) 
    end
end

mutable struct NoiseCache
    noisy_qs 
    noisy_vs
    rand_walks

    function NoiseCache(state)
        new(configuration(state), velocity(state), zeros(6))
    end
end

mutable struct FilterCache
    filtered_vs 
    filtered_state :: MechanismState
    
    function FilterCache(state)
        new(zeros(num_velocities(state.mechanism)), MechanismState(state.mechanism))
    end
end

# ------------------------------------------------------------------------
#                          UTILITY FUNCTIONS
# ------------------------------------------------------------------------
"""
Given a `torque`` value and a torque `limit``, bounds `torque`` to be within the bounds of `limit`.

Assumes the torque limit is the same in the positive and negative directions.
"""
function impose_torque_limit(torque, limit)
    if torque > limit
        new_tau = limit
    elseif torque < -limit
        new_tau = -limit
    else
        new_tau = torque
    end
    new_tau
end

function resettimestep(c::CtlrCache)
    c.step_ctr = 0
end

function skew(x1, x2, x3)
    output = [0 -x3 x2;
            x3 0 -x1;
            -x2 x1 0]
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
function pid_control!(torques::AbstractVector, t, state::MechanismState, pars, c, result, h_wrenches)
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
            torques[7] = -.002 # ff joint E value
            torques[8] = -.32255 # ff Joint D value 
            torques[9] = -.0335 # ff Joint C value
            torques[10] = 0 #.5e-5
            println("At time... ")
        end
        
        # Roll and pitch are not controlled
        torques[1] = 0.
        torques[2] = 0.
        
        # println("Requesting des vel from trajgen")
        # c.des_vel = TrajGen.get_desv_at_t(t, pars)

        # if rem(c.step_ctr, 1000) == 0
        #     @show c.n_cache.rand_walks
        #     # @show result.jointwrenches
        # #     # println("Desired velocity vector: $(c.des_vel)")
        # #     wrist_wrenches = result.totalwrenches[BodyID(wrist_body)]
        # #     wrist_wrench_wrist_frame = transform(wrist_wrenches, relative_transform(state, base_frame, wrist_frame))
        # #     @show wrist_wrench_wrist_frame
        # end

        if rem(c.step_ctr, ctrl_steps) == 0 && c.step_ctr != 0

            if t > swap_times[c.traj_num]
                c.traj_num += 1
            end
            mod_time = c.traj_num == 1 ? t : t-swap_times[c.traj_num-1]
            c.des_vel = get_desv_at_t(mod_time, pars[c.traj_num])
    
            # ff_torques = dynamics_bias(state, h_wrenches)

            noisy_poses = similar(configuration(state))
            noisy_vels = similar(velocity(state))
            fill!(noisy_poses, 0.)
            fill!(noisy_vels, 0.)

            c.n_cache.rand_walks[1:3] .+= rand(gyro_rand_walk_dist, 3)
            c.n_cache.rand_walks[4:6] .+= rand(accel_rand_walk_dist, 3)

            last_noisy_R = Rotations.QuatRotation(c.n_cache.noisy_qs[1:4,end])
            # println("Starting to add noise... Starting noisy poses")
            # @show velocity(state)
            # @show noisy_vels
            add_arm_noise!(noisy_poses, noisy_vels, configuration(state)[8:end], c.n_cache.noisy_qs[8:end,end], 1/ctrl_freq)
            # @show noisy_vels
            new_noisy_R = add_rotational_noise!(noisy_poses, noisy_vels, velocity(state)[1:3], last_noisy_R, 1/ctrl_freq, c.n_cache.rand_walks)
            # @show noisy_vels
            add_linear_noise!(noisy_poses, noisy_vels, result.v̇[1:6], c.last_v̇, c.n_cache.noisy_vs[1:6,end], c.n_cache.noisy_qs[5:7,end], last_noisy_R, new_noisy_R, 1/ctrl_freq, c.n_cache.rand_walks)
            # @show noisy_vels
            # if rem(c.step_ctr, 1000) == 0
            #     @show result.v̇[6]
            #     @show result.accelerations[BodyID(3)]
            # end

            c.last_v̇ = result.v̇
            c.n_cache.noisy_vs = cat(c.n_cache.noisy_vs, noisy_vels, dims=2)
            c.n_cache.noisy_qs = cat(c.n_cache.noisy_qs, noisy_poses, dims=2)

            filtered_velocity = moving_average_filter_velocity(5, c.n_cache.noisy_vs, velocity(state))
            c.f_cache.filtered_vs = cat(c.f_cache.filtered_vs, filtered_velocity, dims=2)
            filtered_vels = similar(velocity(state))
            filtered_vels[:] = filtered_velocity

            set_configuration!(c.f_cache.filtered_state, noisy_poses)
            set_velocity!(c.f_cache.filtered_state, filtered_vels)
            ff_torques = dynamics_bias(c.f_cache.filtered_state, h_wrenches)

            # Get forces for vehicle (yaw, surge, sway, heave)
            for dir_idx = 3:6
                # println("PID ctlr on vehicle")
                actual_vel = velocity(state, joint_dict["vehicle"])
                ctlr_tau = PID_ctlr(torques[dir_idx][1], t, filtered_velocity[dir_idx], dir_idx, c, ff_torques)
                # ctlr_tau = torques[dir_idx]
                c_taus[dir_idx] = ctlr_tau 
                torques[dir_idx] = ctlr_tau
            end
            
            # Get torques for the arm joints
            for idx in 7:length(dof_names) # Joint index (1:vehicle, 2:baseJoint, etc)
                joint_name = dof_names[idx]
                jt_idx = idx-5 # velocity index (7 to 10)
                ctlr_tau = PID_ctlr(torques[idx][1], t, filtered_velocity[idx], idx, c, ff_torques) 
                torques[velocity_range(state, joint_dict[joint_name])] .= [ctlr_tau] 
                c_taus[idx] = ctlr_tau 
            end
            #TODO switch to push! ?
            c.taus = cat(c.taus, c_taus, dims=2)
            # push!(c.taus, copy(c_taus))
        end
    end
    if rem(c.step_ctr, 4000) == 0
        # println("At time $(c.step_ctr/4000)...")
        print("$(c.step_ctr/4000)... ")

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
function PID_ctlr(torque, t, vel_act, idx, c, ff)
    dt = 1/ctrl_freq 
    actuated_idx = idx-2
    dof_name = dof_names[idx]

    d_vel = c.des_vel[actuated_idx]
    vel_error = vel_act[1] - d_vel
    d_vel_error = (vel_error - c.vel_error_cache[actuated_idx])/dt
    # println("D_Velocity error on idx$(j_idx): $(d_vel_error)")
    c.vel_int_error_cache[actuated_idx] += vel_error*dt

    p_term = -Kp_dict[dof_name]*vel_error
    d_term = - Kd_dict[dof_name]*d_vel_error
    i_term = - Ki_dict[dof_name]*c.vel_int_error_cache[actuated_idx]
    d_tau = p_term + d_term + i_term
    # println("Ideal tau: $(d_tau)")

    # Can only change torque a small amount per time step 
    lim = dtau_lim_dict[dof_name]


    tau_diff_prev_to_inv_dyn = .25ff[idx] - .25torque
    d_tau_w_ff = limit_d_tau(tau_diff_prev_to_inv_dyn+d_tau, lim)

    # Torque limits
    if actuated_idx >= 5
        new_tau = torque .+ d_tau_w_ff
        # new_tau = torque .+ d_tau
    else
        d_tau = limit_d_tau(d_tau, lim)
        new_tau = torque .+ d_tau
    end 
    
    new_tau = impose_torque_limit(new_tau, torque_lim_dict[dof_name])

    # if rem(c.step_ctr, 100) == 0 && actuated_idx == 7
    #     @show new_tau
    # end
    # if  t > 12 && rem(c.step_ctr, 100) == 0
    #     @show actuated_idx
    #     @show new_tau
    # end
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

function add_arm_noise!(noisy_poses, noisy_vels, joint_poses, last_noisy_joint_pose, dt)
    noisy_joint_poses = joint_poses + rand(arm_pos_noise_dist, length(joint_poses)) 
    # noisy_joint_poses = joint_poses
    # noisy_joint_poses[2:4] = joint_poses[2:4] + rand(arm_pos_noise_dist, 3)
    noisy_velocity = (noisy_joint_poses - last_noisy_joint_pose)./dt
    noisy_poses[8:end] .= noisy_joint_poses
    noisy_vels[7:end] .= noisy_velocity
end

function add_rotational_noise!(noisy_poses, noisy_vels, body_vels, last_R, dt, rand_walk)
    noisy_vels[1:3] .= body_vels[1:3] + rand(v_ang_vel_noise_dist, 3) + rand_walk[1:3]
    new_R = last_R*exp(skew(noisy_vels[1:3]...)*dt)
    new_noisy_R = Rotations.RotMatrix(SMatrix{3,3}(new_R))
    quat_new_noisy_R = Rotations.QuatRotation(new_noisy_R)
    noisy_poses[1:4] .= [quat_new_noisy_R.w, quat_new_noisy_R.x, quat_new_noisy_R.y, quat_new_noisy_R.z]
    return quat_new_noisy_R
end

function add_linear_noise!(noisy_poses, noisy_vels, v̇, last_v̇, last_body_vels, last_pos, last_R, new_R, dt, rand_walk)
    noisy_linear_accels = last_v̇[4:6] + rand(accel_noise_dist, 3) + rand_walk[4:6]
    noisy_vels[4:6] .= last_body_vels[4:6] + noisy_linear_accels.*dt #+last_v̇[4:6])./2 .*dt
    noisy_vels_in_space = last_R*SVector{3}(last_body_vels[4:6]) #+ new_R*SVector{3}(noisy_vels[4:6]))./2
    noisy_poses[5:7] .= last_pos + noisy_vels_in_space.*dt   
end

function moving_average_filter_velocity(filt_size, noisy_vs, act_vs)
    num_its = size(noisy_vs, 2)
    if num_its < filt_size
        filtered_vels = act_vs
    else
        filtered_vels = sum(noisy_vs[:,end-filt_size+1:end], dims=2) ./ filt_size  
    end
    # @show filtered_vels
    return filtered_vels
end
