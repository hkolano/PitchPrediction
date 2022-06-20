module PDCtlr 
# export ctlr_setup

using RigidBodyDynamics
using PitchPrediction
src_dir = dirname(pathof(PitchPrediction))
traj_file = joinpath(src_dir, "toy-problem", "TrajGen.jl")
include(traj_file)

Kp = 50.
Kd = 2.0
torque_lims = [0.0, 5.0, 5.0]
vel_error_cache = [0.0, 0.0, 0.0]
des_vel = [0.0, 0.0]
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

function pd_control!(torques::AbstractVector, t, state::MechanismState, pars; time_step_ctr=time_step_ctr, joint_vec=joint_vec)
    # If it's the first time called in the Runge-Kutta, update the control torque
    # println("Made it inside the function! Ctr = $(time_step_ctr)")
    if rem(time_step_ctr, 4) == 0
        # println("Div by 4: doing control")
        torques[velocity_range(state, joint_vec[1])] .= -1.0 .* velocity(state, joint_vec[1])
        # println("Requesting des vel from trajgen")
        des_vel = TrajGen.get_desv_at_t(t, pars)
        # println("Got desired velocity")
        for idx in 2:3
            ctlr_tau = PD_ctlr(torques[idx][1], t, velocity(state, joint_vec[idx]), des_vel[idx-1], idx) 
            damp_tau = -.1*velocity(state, joint_vec[idx])
            torques[velocity_range(state, joint_vec[idx])] .= [ctlr_tau] + damp_tau
        end

    end

    global time_step_ctr = time_step_ctr + 1
end;

function PD_ctlr(torque, t, vel_act, des_vel, j_idx; dt=dt)
    vel_error = vel_act[1] - des_vel
    d_vel_error = (vel_error - vel_error_cache[j_idx])/dt
    # println("D_Velocity error on idx$(j_idx): $(d_vel_error)")
    d_tau = -Kp*vel_error - Kd*d_vel_error
    # println("Ideal tau: $(d_tau)")

    # Can only change torque a small amount per time step
    if d_tau < -.1
        d_tau = -0.1
    elseif d_tau > 0.1
        d_tau = 0.1
    end
    
    # Torque limits
    new_tau = torque .+ d_tau
    impose_torque_limit!(new_tau, torque_lims[j_idx])

    # # store velocity error term
    vel_error_cache[j_idx]=vel_error

    return new_tau
end


end