using RigidBodyDynamics, Distributions, Random

include("CtlrParFiles/IrosPitchPredPars.jl")
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
        num_actuated_dofs = num_dofs - 2
        new(n_cache, f_cache, #=
        =# zeros(num_actuated_dofs), zeros(num_actuated_dofs), 0,  #=
        =# zeros(num_actuated_dofs), Array{Float64}(undef, num_dofs, 1), #=
        =#  zeros(num_dofs), 1) 
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
    
    function FilterCache(mechanism)
        new(zeros(num_velocities(mechanism)), MechanismState(mechanism))
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
    if rem(c.step_ctr, 4) == 0
        # Set up empty vector for control torques
        c_taus = zeros(size(c.taus, 1),1)

        # If it's the first step, set the torques to the equilibrium values (set in ConstMagicNums.jl)
        if c.step_ctr == 0
            set_torques_equilibrium!(torques)
            println("At time:")
        end
        
        # Roll and pitch are not controlled
        torques[1] = 0.
        torques[2] = 0.

        # if rem(c.step_ctr, 1000) == 0
        #     @show c.rand_walks
        #     # @show result.jointwrenches
        # #     # println("Desired velocity vector: $(c.des_vel)")
        # #     wrist_wrenches = result.totalwrenches[BodyID(wrist_body)]
        # #     wrist_wrench_wrist_frame = transform(wrist_wrenches, relative_transform(state, base_frame, wrist_frame))
        # #     @show wrist_wrench_wrist_frame
        # end

        if rem(c.step_ctr, ctrl_loop_num_steps) == 0 && c.step_ctr != 0

            # TODO arbitration method for how to get the desired velocity
            # Maybe an input to the controller is the function that should be called?
            if t > swap_times[c.traj_num]
                c.traj_num += 1
            end
            mod_time = c.traj_num == 1 ? t : t-swap_times[c.traj_num-1]
            c.des_vel = get_desv_at_t(mod_time, pars[c.traj_num])
    
            # ff_torques = dynamics_bias(state, h_wrenches)
            noisy_poses = add_sensor_noise!(c, result)
            filtered_vels = filter_velocities!(c, state)

            set_configuration!(c.f_cache.filtered_state, noisy_poses)
            set_velocity!(c.f_cache.filtered_state, filtered_vels)
            ff_torques = dynamics_bias(c.f_cache.filtered_state, h_wrenches)

            # Get forces for vehicle (yaw, surge, sway, heave)
            for dir_idx = 3:6
                joint_name = dof_names[dir_idx]
                # actual_vel = velocity(state, joint_dict["vehicle"])
                filtered_vel = velocity(c.f_cache.filtered_state, joint_dict["vehicle"])
                ctlr_tau = joint_ctlr(torques[dir_idx][1], t, filtered_vel, dir_idx, c, ff_torques, joint_name)
                # ctlr_tau = torques[dir_idx]
                c_taus[dir_idx] = ctlr_tau 
                torques[dir_idx] = ctlr_tau
            end
            
            # Get torques for the arm joints
            for jt_idx in 2:length(c.joint_vec) # Joint index (1:vehicle, 2:baseJoint, etc)
                idx = jt_idx+5 # velocity index (7 to 10)
                ctlr_tau = joint_ctlr(torques[idx][1], t, filtered_velocity[idx], idx, c, ff_torques) 
                torques[velocity_range(state, c.joint_vec[jt_idx])] .= [ctlr_tau] 
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
function get_feedback_term(c, actuated_idx, measured_vel)
    dt = 1/ctrl_freq
    # TODO change c.des_vel to be a dictionary
    d_vel = c.des_vel[actuated_idx]
    vel_error = measured_vel[1] - d_vel 
    d_vel_error = (vel_error - c.vel_error_cache[actuated_idx])/dt 
    c.vel_int_error_cache[actuated_idx] += vel_error*dt

    p_term = -Kp[actuated_idx]*vel_error
    d_term = - Kd[actuated_idx]*d_vel_error
    i_term = - Ki[actuated_idx]*c.vel_int_error_cache[actuated_idx]
    d_tau = p_term + d_term + i_term
end

function get_feedforward_term(ff, idx, torque)
    # ff_prop is set in CtlrParFiles
    feedforward_dtau = ff_prop*ff[idx] - ff_prop*torque
end

function joint_ctlr(torque, t, vel_act, idx, c, ff, joint_name)
    # PID feedback term
    feedback_dtau = get_feedback_term(c, idx-2, vel_act)
    # If using feedforward, calc feedforward term
    do_feedforward == true ? feedforward_dtau = get_feedforward_term(ff, idx, torque) : 0.0

    # Can only change torque a small amount per time step 
    # arm joints can change faster than thrusters
    #TODO if ever change controller or actual time step, change limits
    lim = dtau_lim_dict[joint_name]   
    dtau = limit_d_tau(tau_diff_prev_to_inv_dyn+d_tau, lim)

    # Torque limits
    if actuated_idx >= 5
        new_tau = torque .+ d_tau_w_ff
        # new_tau = torque .+ d_tau
    else
        d_tau = limit_d_tau(d_tau, lim)
        new_tau = torque .+ d_tau
    end 
    
    new_tau = impose_torque_limit(new_tau, torque_lims[actuated_idx])

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