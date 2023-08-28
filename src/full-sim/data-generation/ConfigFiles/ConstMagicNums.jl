# ----------------------------------------------------------
# World information
# ----------------------------------------------------------
rho = 997 # kg/m^3 (density of the water)

# ----------------------------------------------------------
# Simulation information
# ----------------------------------------------------------
Δt = 1.e-3          # simulation time step
ctrl_freq = Int(100)     # frequency of call to controller
ctrl_steps = 4*(1/Δt)/ctrl_freq # number of steps to take before changing controller action
plot_freq = Int(50)      # frequency of the output CSV and plots
plot_factor = ctrl_freq/plot_freq
sample_rate = Int(floor((1/Δt)/plot_freq))
ctrl_loop_num_steps = 4*(1/Δt)/ctrl_freq

@assert rem(ctrl_loop_num_steps, 1) == 0.0 "Please pick a control frequency that is an even divisor of the simulation frequency."
@assert rem(plot_factor, 1) == 0.0 "Please pick a plot frequency that is an even divisor of the control frequency."

# ----------------------------------------------------------
# Truly magic numbers
# ----------------------------------------------------------
num_its = 50        # number of points to check that the desired joint position and velocity limits don't go out of range
max_traj_duration = 20. # maximum number of seconds for a quintic trajectory to take

# ----------------------------------------------------------
# System-specific functions and information
# ----------------------------------------------------------
                     
# equilibrium of unmodified ROV + Alpha arm
function reset_to_equilibrium!(state)
    zero!(state)
    set_configuration!(state, joint_dict["vehicle"], [.9777, -0.0019, 0.2098, .0079, 0., 0., 0.])
end

function reset_to_equilibrium_hardware!(state)
    zero!(state)
    set_configuration!(state, joint_dict["vehicle"], [0.9995, 0.02034, 0.02448, .00019, 0., 0., 0.])
end

# FF torques to station keep at equilibrium
# function set_torques_equilibrium!(torques)
#     torques[1] = 0.     # roll
#     torques[2] = 0.     # pitch
#     torques[3] = 0.     # yaw
#     torques[4] = -2.3   # vehicle X 
#     torques[5] = 0.0    # Vehicle Y 
#     torques[6] = 5.2    # Vehicle Z
#     torques[7] = -.002  # Base joint (Joint E)
#     torques[8] = -.32255 # Shoulder joint (Joint D)
#     torques[9] = -0335  # Elbow joint (Joint C)
#     torques[10] = 0.0   # wrist joint (Joint B)
#     if length(torques) > 10
#         torques[11] = 0.0   # jaw joint (Joint A)
#     end
# end


# Sensor noise distributions 
# Encoder --> joint position noise -integration-> joint velocity noise
# Gyroscope --> vehicle body vel noise 
v_ang_vel_noise_dist = Distributions.Normal(0, .0013) # 75 mdps (LSM6DSOX)
arm_pos_noise_dist = Distributions.Normal(0, .0017/6) # .1 degrees, from Reach website
accel_noise_dist = Distributions.Normal(0, 0.017658/10) # 1.8 mg = .0176 m/s2 (LSM6DSOX)

gyro_rand_walk_dist = Distributions.Normal(0, .000001)
accel_rand_walk_dist = Distributions.Normal(0, 0)#0.00001)
