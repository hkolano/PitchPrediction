using RigidBodyDynamics, Distributions, Random

# ------------------------------------------------------------------------
#                              SETUP 
# ------------------------------------------------------------------------
arm_Kp = 30.
arm_Kd = 0.05
arm_Ki = 9. 
v_Kp = .18 
v_Kd = 0.0001 
v_Ki = 0.012
Kp = [50, v_Kp, v_Kp, v_Kp, arm_Kp, arm_Kp, arm_Kp, .03, 20.]
Kd = [.5, v_Kd, v_Kd, v_Kd, arm_Kd, arm_Kd, arm_Kd, 0.0001, .002]
Ki = [1.5, v_Ki, v_Ki, v_Ki, arm_Ki, arm_Ki, arm_Ki, 0.0001, .1]
torque_lims = [20., 71.5, 88.2, 177., 10.0, 10.0, 10.0, 0.6, 600]

# Sensor noise distributions 
v_ang_vel_noise_dist = Distributions.Normal(0, .01)
v_lin_vel_noise_dist = Distributions.Normal(0, .01)
arm_vel_noise_dist = Distributions.Normal(0, .001)

v_ori_noise_dist = Distributions.Normal(0, .001)
v_pos_noise_dist = Distributions.Normal(0, .001)
arm_pos_noise_dist = Distributions.Normal(0, .001)

mutable struct CtlrCache
    time_step::Float64
    vel_error_cache::Array{Float64}
    vel_int_error_cache::Array{Float64}
    step_ctr::Int
    joint_vec
    des_vel::Array{Float64}
    taus
    CLIK_gains::Array{Float64}
    des_zetas
    
    function CtlrCache(dt, mechanism)
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
        new(dt, zeros(num_actuated_dofs), zeros(num_actuated_dofs), 0, joint_vec, #=
        =# zeros(num_actuated_dofs), Array{Float64}(undef, num_dofs, 1)) 
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
function pid_control!(torques::AbstractVector, t, state::MechanismState, pars, c)
# function pid_control!(torques::AbstractVector, t, state::MechanismState, traj, c)
    # If it's the first time called in the Runge-Kutta, update the control torque
    # println("Made it inside the function! Ctr = $(time_step_ctr)")
    if rem(c.step_ctr, 4) == 0
        # Set up empty vector for control torques
        c_taus = zeros(size(c.taus, 1),1)
        if c.step_ctr == 0
            torques[6] = 5.2 # ff z value
            torques[3] = 0.
            torques[5] = 0.
            torques[4] = -2.3 # ff x value
            torques[7] = .02 # ff joint E value
            torques[8] = -.30 # ff Joint D value 
            torques[9] = -.035 # ff Joint C value
            torques[10] = .003
        end
        
        # Roll and pitch are not controlled
        torques[1] = 0.
        torques[2] = 0.

        if 4 <= t <= 4.0015 
            println("hit 4 s")
        end
        
        # println("Requesting des vel from trajgen")
        # c.des_vel = TrajGen.get_desv_at_t(t, pars)
        c.des_vel = get_desv_at_t(t, pars)
        if rem(c.step_ctr, 1000) == 0
            println("Desired velocity vector: $(c.des_vel)")
        end
        # Don't move the manipulator
        # c.des_vel[end-3:end] = zeros(4,1)

        noisy_velocity = add_velocity_noise(velocity(state))

        # Get forces for vehicle (yaw, surge, sway, heave)
        for dir_idx = 3:6
            # println("PID ctlr on vehicle")
            actual_vel = velocity(state, c.joint_vec[1])
            ctlr_tau = PID_ctlr(torques[dir_idx][1], t, noisy_velocity[dir_idx], dir_idx, c)
            c_taus[dir_idx] = ctlr_tau 
            torques[dir_idx] = ctlr_tau
        end
        
        # Get torques for the arm joints
        for jt_idx in 2:length(c.joint_vec) # Joint index (1:vehicle, 2:baseJoint, etc)
            idx = jt_idx+5 # velocity index (7 to 10)
            ctlr_tau = PID_ctlr(torques[idx][1], t, noisy_velocity[idx], idx, c) 
            torques[velocity_range(state, c.joint_vec[jt_idx])] .= [ctlr_tau] 
            c_taus[idx] = ctlr_tau 
        end
        #TODO switch to push! ?
        c.taus = cat(c.taus, c_taus, dims=2)
        # push!(c.taus, copy(c_taus))
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
    d_vel_error = (vel_error - c.vel_error_cache[actuated_idx])/c.time_step
    # println("D_Velocity error on idx$(j_idx): $(d_vel_error)")
    c.vel_int_error_cache[actuated_idx] = c.vel_int_error_cache[actuated_idx] + vel_error*c.time_step
    d_tau = -Kp[actuated_idx]*vel_error - Kd[actuated_idx]*d_vel_error - Ki[actuated_idx]*c.vel_int_error_cache[actuated_idx]
    # println("Ideal tau: $(d_tau)")

    # Can only change torque a small amount per time step 
    # arm joints can change faster than thrusters
    5 <= actuated_idx ? lim = 0.001 : lim = 0.01
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
    noisy_velocity[7:end] = velocity[7:end] + rand(arm_vel_noise_dist, length(velocity[7:end]))
    return noisy_velocity
end
