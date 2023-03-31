using RigidBodyDynamics, Distributions, Random

include("CtlrParFiles/IrosPitchPredPars.jl")
# ------------------------------------------------------------------------
#                              SETUP 
# ------------------------------------------------------------------------
mutable struct CtlrCache
    n_cache 
    f_cache
    vel_error_cache::Dict{String, Float64}
    vel_int_error_cache::Dict{String, Float64}
    step_ctr::Int
    des_vel::Dict{String, Float64}
    taus
    last_v̇
    
    function CtlrCache(state, n_cache, f_cache)
        mechanism = state.mechanism
        num_dofs = num_velocities(mechanism)
        num_actuated_dofs = num_dofs-2

        vel_error_cache_dict = Dict{String, Float64}()
        vel_int_error_cache_dict = Dict{String, Float64}()
        des_vel_dict = Dict{String, Float64}()
        for dof in dof_names
            vel_error_cache_dict[dof] = 0.0
            vel_int_error_cache_dict[dof] = 0.0
            des_vel_dict[dof] = 0.0
        end


        new(n_cache, f_cache, 
            vel_error_cache_dict, vel_int_error_cache_dict, 0, 
            des_vel_dict, Array{Float64}(undef, num_dofs, 1), 
            zeros(num_dofs)) 
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

        # Set torques to appropriate values for equilibrium position
        if c.step_ctr == 0
            set_equilibrium_torques!(torques)
            println("At time... ")
        end
        
        # Roll and pitch are not controlled
        torques[1] = 0.
        torques[2] = 0.
        
        # if rem(c.step_ctr, 1000) == 0
        #     @show c.n_cache.rand_walks
        #     # @show result.jointwrenches
        # #     # println("Desired velocity vector: $(c.des_vel)")
        # #     wrist_wrenches = result.totalwrenches[BodyID(wrist_body)]
        # #     wrist_wrench_wrist_frame = transform(wrist_wrenches, relative_transform(state, base_frame, wrist_frame))
        # #     @show wrist_wrench_wrist_frame
        # end

        if rem(c.step_ctr, ctrl_steps) == 0 && c.step_ctr != 0

            manip_des_vels = get_desv_at_t(t, pars)
            for (idx, dof_name) in enumerate(dof_names[7:end])
                c.des_vel[dof_name] = manip_des_vels[idx]
            end
            #
            noisy_poses, noisy_vels = add_sensor_noise(state, c, result)
            filtered_vels = filter_velocity(state, c) 

            set_configuration!(c.f_cache.filtered_state, noisy_poses)
            set_velocity!(c.f_cache.filtered_state, filtered_vels[:])
            ff_torques = dynamics_bias(c.f_cache.filtered_state, h_wrenches)
            ff_torques[1:6] .= zeros(6)

            for dir_idx = 3:num_dofs
                dof = dof_names[dir_idx]

                meas_vel = filtered_vels[dir_idx]
                ctlr_tau = joint_controller(torques[dir_idx][1], filtered_vels[dir_idx], dof, c, ff_torques[dir_idx])

                c_taus[dir_idx] = ctlr_tau
                torques[dir_idx] = ctlr_tau
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
function joint_controller(torque, vel_act, dof_name, c, ff)
    dt = 1/ctrl_freq 

    # Calculate proportional term
    vel_error = vel_act[1] - c.des_vel[dof_name]
    p_term = -Kp_dict[dof_name]*vel_error

    # Calculate derivative term 
    d_vel_error = (vel_error - c.vel_error_cache[dof_name])/dt
    d_term = - Kd_dict[dof_name]*d_vel_error
    c.vel_error_cache[dof_name]=vel_error

    # Calculate integral term
    c.vel_int_error_cache[dof_name] += vel_error*dt
    i_term = - Ki_dict[dof_name]*c.vel_int_error_cache[dof_name]

    d_tau = p_term + d_term + i_term
    # println("Feedback tau: $(d_tau)")

    # Can only change torque a small amount per time step 
    tau_diff_prev_to_inv_dyn = .25ff - .25torque
    d_tau_w_ff = limit_d_tau(tau_diff_prev_to_inv_dyn+d_tau, dtau_lim_dict[dof_name])
    
    # impose maximum torque limits
    new_tau = torque .+ d_tau_w_ff
    new_tau = impose_torque_limit(new_tau, torque_lim_dict[dof_name])
 
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
