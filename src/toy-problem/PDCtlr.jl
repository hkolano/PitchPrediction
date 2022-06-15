module PDCtlr 

using RigidBodyDynamics

Kp = 50.
Kd = 2.0
torque_lims = (joint1=5.0, joint2=5.0)
vel_error_cache = (joint1=[0.0], joint2=[0.0])
# ------------------------------------------------------------------------
#                          SETUP FUNCTIONS
# ------------------------------------------------------------------------
    function ctlr_setup(mechanism, state; time_step=1e-2,)
        set_ctlr_time_step(time_step)
        resettimestep()
        set_mechanism_to_control(mechanism)
        set_first_state(state)
    end

    function set_ctlr_time_step(time_step)
        global dt = time_step
    end

    function resettimestep()
        global time_step_ctr = 0
    end

    function set_mechanism_to_control(mechanism)
        global ctlr_mechanism = mechanism
        global base_joint, shoulder_joint, elbow_joint, wrist_joint = joints(ctlr_mechanism)
    end

    function set_first_state(state)
        global ctlr_first_state = state 
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

function simple_control!(torques::AbstractVector, t, state_alpha::MechanismState, hydro_wrenches::Dict{BodyID, Wrench{Float64}})
    # If it's the first time called in the Runge-Kutta, update the control torque
    if rem(time_step_ctr, 4) == 0
        # println("New control goes here!!!")

        # Get the desired velocity values for each joint
        get_des_vel!(des_vel, t)

        tau_base = MyCtlr.PD_ctlr(torques[velocity_range(state_alpha, base_joint)][1], t, velocity(state_alpha, base_joint), des_vel[base_joint], 1)
        tau_shoulder = MyCtlr.PD_ctlr(torques[velocity_range(state_alpha, shoulder_joint)][1], t, velocity(state_alpha, shoulder_joint), des_vel[shoulder_joint], 2)
        tau_elbow = MyCtlr.PD_ctlr(torques[velocity_range(state_alpha, elbow_joint)][1], t, velocity(state_alpha, elbow_joint), des_vel[elbow_joint], 3)
        tau_wrist = MyCtlr.PD_ctlr(torques[velocity_range(state_alpha, wrist_joint)][1], t, velocity(state_alpha, wrist_joint), des_vel[wrist_joint], 4)

        torques[velocity_range(state_alpha, base_joint)] .= tau_base
        torques[velocity_range(state_alpha, shoulder_joint)] .= tau_shoulder #tau[shoulder_joint][1]
        torques[velocity_range(state_alpha, elbow_joint)] .= tau_elbow #0.0 #tau[elbow_joint][1]
        torques[velocity_range(state_alpha, wrist_joint)] .= tau_wrist #tau[wrist_joint][1]
    end

    # push!(ctlr_taus, tau_base)
    # push!(times, t)

    global time_step_ctr += 1
    # println("Base joint vel errors: ", vel_error_cache.base_joint)
end;

function get_des_vel!(des_vel, t)
    # TODO: implement a simple pt a to pt b trajectory generation
    des_vel[base_joint][1] = 1.0
    des_vel[shoulder_joint][1] = 0.0
    des_vel[elbow_joint][1] = 0.0 #vdot[elbow_joint][1]
    des_vel[wrist_joint][1] = 0.00
end;
    
function PD_ctlr(torque, t, vel_act, des_vel, j_idx)
    vel_error = vel_act[1] - des_vel[1]
    d_vel_error = (vel_error - last(vel_error_cache[j_idx]))/dt
    d_tau = -Kp*vel_error - Kd*d_vel_error

    # Can only change torque a small amount per time step
    if d_tau < -.05
        d_tau = -0.05
    elseif d_tau > 0.05
        d_tau = 0.05
    end
    
    # Torque limits
    new_tau = torque .+ d_tau
    impose_torque_limit!(new_tau, torque_lims[j_idx])

    # store velocity error term
    push!(vel_error_cache[j_idx], vel_error)

    return new_tau
end


end