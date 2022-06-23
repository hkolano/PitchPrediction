module PIDwFFCtlr 
export CtlrCache, pidff_control!

using RigidBodyDynamics
using PitchPrediction

src_dir = dirname(pathof(PitchPrediction))
traj_file = joinpath(src_dir, "toy-problem", "TrajGen.jl")
include(traj_file)

# ------------------------------------------------------------------------
#                              SETUP 
# ------------------------------------------------------------------------

default_Kp = 50000.
default_Kd = 20.0
default_Ki = 20.0
default_torque_lims = [0.0, 5.0, 5.0]

mutable struct CtlrCache
    Kp::Float64
    Kd::Float64
    Ki::Float64
    time_step::Float64
    vel_error_cache::Array{Float64}
    vel_int_error::Float64
    step_ctr::Int
    joint_vec
    des_vel::Array{Float64}
    tau_lims::Array{Float64}
    taus
    last_vel

    function CtlrCache(dt, mechanism)
        free_joint, joint1, joint2 = joints(mechanism)
        joint_vec = [free_joint, joint1, joint2]
        new(default_Kp, default_Kd, default_Ki, dt, [0.0, 0.0], 0., 0, joint_vec, [0.0, 0.0], default_torque_lims, Array{Float64}(undef, 2, 1), [0.0, 0.0, 0.0]) 
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
Imposes a PD controller to follow a velocity specified by TrajGen.get_desv_at_t().
In this case, also imposes damping to each joint. 
Requires the parameters of the trajectory to be followed (pars=trajParams), which consists of the quintic coefficients `a` and the two waypoints to travel between.
Only happens every 4 steps because integration is done with Runge-Kutta.
"""
function pidff_control!(torques::AbstractVector, t, state::MechanismState, pars, c)
    # If it's the first time called in the Runge-Kutta, update the control torque
    # println("Made it inside the function! Ctr = $(time_step_ctr)")
    if rem(c.step_ctr, 4) == 0
        c_taus = [0.0; 0.0]
        # println("Div by 4: doing control")
        torques[velocity_range(state, c.joint_vec[1])] .= -1.0 .* velocity(state, c.joint_vec[1])
        # println("Requesting des vel from trajgen")
        c.des_vel = TrajGen.get_desv_at_t(t, pars)
        des_acc = similar(velocity(state))
        est_acc_pitch = (velocity(state, c.joint_vec[1])[1] - c.last_vel[1])/c.time_step
        des_acc[2:3] = c.des_vel .- velocity(state)[2:3]
        des_acc[1] = est_acc_pitch
        # println("Desired acc: ($des_acc)")
        ff_tau = inverse_dynamics(state, des_acc)
        println("Inverse Dynamics tau = $(ff_tau)")
        # println("Got desired velocity")
        for idx in 2:3
            ctlr_tau = PID_ctlr(torques[idx][1], t, velocity(state, c.joint_vec[idx]), idx, c) 
            c_taus[idx-1] = ctlr_tau
            damp_tau = -0.1*velocity(state, c.joint_vec[idx])
            torques[velocity_range(state, c.joint_vec[idx])] .= [ctlr_tau] + damp_tau + [0.25*ff_tau[idx]]
        end
        c.taus = cat(c.taus, c_taus, dims=2)
        c.last_vel = velocity(state)
    end
    c.step_ctr = c.step_ctr + 1
end;

"""
Imposes a PD controller on one joint. 

`torque` = storage variable, does not modify
`t` = current time. Not in use. 
`vel_act` = the actual instantaneous velocity of the joint 
`des_vel` = the desired velocity of the joint, as found by the trajectory generator
`j_idx` = the index of the joint (of the actuated ones). Here, will be 1 or 2.

Returns a controller torque value, bounded by the torque limits and some dτ/dt value. 
"""
function PID_ctlr(torque, t, vel_act, j_idx, c)
    d_vel = c.des_vel[j_idx-1]
    vel_error = vel_act[1] - d_vel
    d_vel_error = (vel_error - c.vel_error_cache[j_idx-1])/c.time_step
    # println("D_Velocity error on idx$(j_idx): $(d_vel_error)")
    c.vel_int_error = c.vel_int_error + vel_error*c.time_step
    d_tau = -c.Kp*vel_error - c.Kd*d_vel_error - c.Ki*c.vel_int_error
    # println("Ideal tau: $(d_tau)")

    # Can only change torque a small amount per time step
    if d_tau < -.05
        d_tau = -0.05
    elseif d_tau > 0.05
        d_tau = 0.05
    end
    
    # Torque limits
    new_tau = torque .+ d_tau
    impose_torque_limit!(new_tau, c.tau_lims[j_idx])

    # # store velocity error term
    c.vel_error_cache[j_idx-1]=vel_error
    # c.vel_int_error = c.vel_int_error + vel_error

    return new_tau
end


end