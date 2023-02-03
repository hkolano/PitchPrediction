# ------------------------------------------------------------------------
#                              SETUP 
# ------------------------------------------------------------------------
arm_Kp = 30.
arm_Kd = 0.05
arm_Ki = 9. 
v_Kp = 100.
v_Kd = 1.0
v_Ki = 1.25
default_Kp = [v_Kp, v_Kp, v_Kp, v_Kp, arm_Kp, arm_Kp, arm_Kp, 20., 20.]
default_Kd = [v_Kd, v_Kd, v_Kd, v_Kd, arm_Kd, arm_Kd, arm_Kd, 0.002, .002]
default_Ki = [v_Ki, v_Ki, v_Ki, v_Ki, arm_Ki, arm_Ki, arm_Ki, 0.1, .1]
default_torque_lims = [20., 71.5, 88.2, 177., 10.0, 10.0, 10.0, 0.6, 600]
CLIK_gains = diagm([1., 1., 1., 1., 1., 1.])

mutable struct CtlrCache
    Kp::Array{Float64}
    Kd::Array{Float64}
    Ki::Array{Float64}
    time_step::Float64
    vel_error_cache::Array{Float64}
    vel_int_error_cache::Array{Float64}
    step_ctr::Int
    joint_vec
    des_vel::Array{Float64}
    tau_lims::Array{Float64}
    taus
    CLIK_gains::Array{Float64}
    des_zetas
    
    function CtlrCache(dt, mechanism)
        vehicle_joint, jointE, jointD, jointC, jointB, jointJaw = joints(mechanism)
        joint_vec = [vehicle_joint, jointE, jointD, jointC, jointB, jointJaw]
        new(default_Kp, default_Kd, default_Ki, #=
        =# dt, zeros(9), zeros(9), 0, joint_vec, #=
        =# zeros(9), default_torque_lims, Array{Float64}(undef, 11, 1), #=
        =# CLIK_gains, Array{Float64}(undef, 11, 1)) 
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

function get_zeta(traj, t, state, c)
    jacobians = []
    ζs = []
    
    # Task 1: Underactuation
    J1 = hcat(diagm([1., 1., 0., 0., 0., 0.,]), zeros(6,4))
    vehicle_joint = joints(state.mechanism)[1]
    des_state = velocity(state, vehicle_joint)
    ζ1 = get_mp_pinv(J1)*des_state

    push!(jacobians, J1)
    push!(ζs, ζ1)

    # Task 2: End Effector Pose
    J2 = calculate_actuated_jacobian(state)
    task_err, ff_vel = get_ee_task_error_and_ff_vel(traj, t, state)
    println(t, task_err)
    # task_err = get_ee_task_error(traj, t, state)
    aug_J2 = cat(J1, J2, dims=1)
    aug_N1 = I - get_mp_pinv(J1)*J1
    
    ζ2 = ζ1 + get_mp_pinv(J2*aug_N1)*(c.CLIK_gains*task_err - J2*ζ1)
    # ζ2 = ζ1 + get_mp_pinv(J2*aug_N1)*(ff_vel + c.CLIK_gains*task_err - J2*ζ1)
    # ζ2 = ζ1 + get_mp_pinv(J2*aug_N1)*(ff_vel - J2*ζ1)
    # ζ = ζ1 + aug_N1*ζ2

    # push!(jacobians, J2)
    # push!(ζs, ζ2)
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
function pid_control!(torques::AbstractVector, t, state::MechanismState, traj, c)
    # If it's the first time called in the Runge-Kutta, update the control torque
    # println("Made it inside the function! Ctr = $(time_step_ctr)")
    if rem(c.step_ctr, 4) == 0
        # Set up empty vector for control torques
        c_taus = zeros(11,1)
        if c.step_ctr == 0
            torques[6] = 5.2 # 7
            torques[3] = 0.
            torques[5] = 0.
            torques[4] = -2.3 # -1
        end
        
        # Roll and pitch are not controlled
        torques[1] = 0.
        torques[2] = 0.

        if 4 <= t <= 4.0015 
            println("hit 4 s")
        end
        
        # println("Requesting des vel from trajgen")
        ζ = get_zeta(traj, t, state, c)

        c.des_vel = vcat(ζ[3:end], 0)
        c.des_zetas = cat(c.des_zetas, vcat(ζ, 0.), dims=2)
        # c.des_vel = [0., 1., 0, 0, 0, 0, 0, 0, 0]

        # Get forces for vehicle (yaw, surge, sway, heave)
        for dir_idx = 3:6
            # println("PID ctlr on vehicle")
            ctlr_tau = PID_ctlr(torques[dir_idx][1], t, velocity(state, c.joint_vec[1])[dir_idx], dir_idx, c)
            c_taus[dir_idx] = ctlr_tau 
            torques[dir_idx] = ctlr_tau
        end
        
        # Get torques for the arm joints
        for jt_idx in 2:6 # Joint index (1:vehicle, 2:baseJoint, etc)
            # println("PID ctlr on arm")
            idx = jt_idx+5 # velocity index 
            ctlr_tau = PID_ctlr(torques[idx][1], t, velocity(state, c.joint_vec[jt_idx]), idx, c) 
            c_taus[jt_idx+5] = ctlr_tau
            damp_tau = -0.1*velocity(state, c.joint_vec[jt_idx])
            torques[velocity_range(state, c.joint_vec[jt_idx])] .= [ctlr_tau] + damp_tau
        end
        c.taus = cat(c.taus, c_taus, dims=2)
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
    d_vel_error = (vel_error - c.vel_error_cache[actuated_idx])/c.time_step
    # println("D_Velocity error on idx$(j_idx): $(d_vel_error)")
    c.vel_int_error_cache[actuated_idx] = c.vel_int_error_cache[actuated_idx] + vel_error*c.time_step
    d_tau = -c.Kp[actuated_idx]*vel_error - c.Kd[actuated_idx]*d_vel_error - c.Ki[actuated_idx]*c.vel_int_error_cache[actuated_idx]
    # println("Ideal tau: $(d_tau)")

    # Can only change torque a small amount per time step
    if 5 <= actuated_idx
        # if it's an arm joint
        lim = 0.02
    else
        # if it's a vehicle "thruster"
        lim = 0.001
    end
    d_tau = limit_d_tau(d_tau, lim)
    
    # Torque limits
    new_tau = torque .+ d_tau
    impose_torque_limit!(new_tau, c.tau_lims[actuated_idx])

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