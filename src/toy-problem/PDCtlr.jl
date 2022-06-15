module PDCtlr 
# export ctlr_setup

using RigidBodyDynamics

Kp = 50.
Kd = 2.0
torque_lims = [0.0, 5.0, 5.0]
vel_error_cache = [0.0, 0.0, 0.0]
# ------------------------------------------------------------------------
#                          SETUP FUNCTIONS
# ------------------------------------------------------------------------
function ctlr_setup(mechanism::Mechanism{Float64}, state::MechanismState; time_step=1e-2)
    global dt = time_step
    resettimestep()
    set_first_state(state)
    free_joint, joint1, joint2 = joints(mechanism)
    global joint_vec = [free_joint, joint1, joint2]
end

function resettimestep()
    global time_step_ctr = 0
end

function set_first_state(state)
    ctlr_first_state = state 
    global des_vel = similar(velocity(ctlr_first_state))
    global prev_vels = similar(des_vel)
end

# ------------------------------------------------------------------------
#                          UTILITY FUNCTIONS
# ------------------------------------------------------------------------
function impose_torque_limit!(torque, limit)
    if torque > limit
        torque = limit
    elseif torque < -limit
        torque = -limit
    end
end

# ------------------------------------------------------------------------
#                              CONTROLLER
# ------------------------------------------------------------------------

function pd_control!(torques::AbstractVector, t, state::MechanismState; time_step_ctr=time_step_ctr, joint_vec=joint_vec)
    # If it's the first time called in the Runge-Kutta, update the control torque
    (j1, j2, j3) = joint_vec
    if rem(time_step_ctr, 4) == 0
        # println("New control goes here!!!")

        torques[velocity_range(state, j1)] .= -.1 .* velocity(state, j1)
        # torques[velocity_range(state, j2)] .= -.1 .* velocity(state, j2)
        # torques[velocity_range(state, j3)] .= -.1 .* velocity(state, j3)

        get_des_vel!(des_vel, t)

        for idx in 2:3
            # println("at idx $(idx)")
            torques[idx] = PD_ctlr(torques[idx][1], t, velocity(state, joint_vec[idx]), des_vel[idx], idx) - .1 .*velocity(state, joint_vec[idx])
        end

    end

    # push!(ctlr_taus, tau_base)
    # push!(times, t)

    time_step_ctr += 1
    # println("Base joint vel errors: ", vel_error_cache.base_joint)
end;

function get_des_vel!(des_vel, t; joint_vec=joint_vec)
    # TODO: implement a simple pt a to pt b trajectory generation
    (~, j2, j3) = joint_vec
    des_vel[j2][1] = 1.0
    des_vel[j3][1] = 0.0
end;
    
function PD_ctlr(torque, t, vel_act, des_vel, j_idx; dt=dt)
    vel_error = vel_act[1] - des_vel[1]
    d_vel_error = (vel_error - vel_error_cache[j_idx])/dt
    # println("D_Velocity error on idx$(j_idx): $(d_vel_error)")
    d_tau = -Kp*vel_error - Kd*d_vel_error
    # println("Ideal tau: $(d_tau)")

    # Can only change torque a small amount per time step
    if d_tau < -.05
        d_tau = -0.05
    elseif d_tau > 0.05
        d_tau = 0.05
    end
    
    # Torque limits
    new_tau = torque .+ d_tau
    impose_torque_limit!(new_tau, torque_lims[j_idx])

    # # store velocity error term
    vel_error_cache[j_idx]=vel_error

    return new_tau
end


end