# Controller parameters used for the pitch prediction trials for the IROS redo. 
# Uses 25% feedforward term.
# TODO add jaw controller to this
v_Kp = 1.2 #3.
v_Ki = 0.6 #3.6
v_Kd = 1.61 #1.68

# Classic
Kp = 0.6
Ki = 1.2
Kd = .075

# Pessen Integral 
# Kp = .7
# Ki = 1.75
# Kd = 0.105

# some overshoot 
# Kp = .333
# Ki = .667
# Kd = .111

# no overshoot 
# Kp = .2
# Ki = .4
# Kd = .0667

Ku_wrist = 1.35e-3
Tu_wrist = 0.16

Ku_elbow = .06
Tu_elbow = .3

Ku_shoulder = .2
Tu_shoulder = .3

Ku_base = .035
Tu_base = .3


Kp_dict = Dict("yaw" => 2.0, 
                "x" => v_Kp, 
                "y" => v_Kp,
                "z" => v_Kp,
                "base" => 3.38e-2,
                "shoulder" => 4.59e-2,
                "elbow" => 1.35e-2,
                # "base" => Kp*Ku_base,
                # "shoulder" => Kp*Ku_shoulder,
                # "elbow" => Kp*Ku_elbow,
                # "elbow" => Ku_elbow,
                "wrist" => Kp*Ku_wrist, 
                # "wrist" => 1.35e-3,
                "jaw" => 5.4e-4)
Ki_dict = Dict("yaw" => 1., 
                "x" => v_Ki, 
                "y" => v_Ki,
                "z" => v_Ki,
                "base" => 4.39e-1,
                "shoulder" => 3.73e-1,
                "elbow" => 9.64e-2,
                # "base" => Ki*Ku_base/Tu_base,
                # "shoulder" => Ki*Ku_shoulder/Tu_shoulder,
                # "elbow" => Ki*Ku_elbow/Tu_elbow,
                "wrist" => Ki*Ku_wrist/Tu_wrist, 
                # "wrist" => 0.0,
                "jaw" => 3.6e-3)
Kd_dict = Dict("yaw" => 0.1, 
                "x" => v_Kd, 
                "y" => v_Kd,
                "z" => v_Kd,
                "base" => 1.74e-3,
                "shoulder" => 3.82e-3,
                "elbow" => 1.27e-3,
                # "base" => Kd*Ku_base*Tu_base,
                # "shoulder" => Kd*Ku_shoulder*Tu_shoulder,
                # "elbow" => Kd*Ku_elbow*Tu_elbow,
                "wrist" => Kd*Ku_wrist*Tu_wrist,
                # "wrist" => 0.0, 
                "jaw" => 2.03e-5)

do_feedforward = true
ff_prop = 0.25

filtering_kernel = 5

function set_equilibrium_torques!(torques)
    torques[3] = 0.         # ff yaw value
    torques[4] = -2.3       # ff x value
    torques[5] = 0.         # ff y value
    torques[6] = 5.2        # ff z value

    torques[7] = -.002      # ff joint E value
    torques[8] = -.32255    # ff Joint D value 
    torques[9] = -.0335     # ff Joint C value
    torques[10] = 0         #.5e-5
end
