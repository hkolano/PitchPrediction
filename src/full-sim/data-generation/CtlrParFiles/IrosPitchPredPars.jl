# Controller parameters used for the pitch prediction trials for the IROS redo. 
# Uses 25% feedforward term.
# TODO add jaw controller to this
v_Kp = 1.2 #3.
v_Ki = 0.6 #3.6
v_Kd = 1.61 #1.68
Kp_dict = Dict("yaw" => 2.0, 
                "x" => v_Kp, 
                "y" => v_Kp,
                "z" => v_Kp,
                "base" => 3.38e-2,
                "shoulder" => 4.59e-2,
                "elbow" => 1.35e-2,
                "wrist" => 5.4e-4)
Ki_dict = Dict("yaw" => 1., 
                "x" => v_Ki, 
                "y" => v_Ki,
                "z" => v_Ki,
                "base" => 4.39e-1,
                "shoulder" => 3.73e-1,
                "elbow" => 9.64e-2,
                "wrist" => 3.6e-3)
Kd_dict = Dict("yaw" => 0.1, 
                "x" => v_Kd, 
                "y" => v_Kd,
                "z" => v_Kd,
                "base" => 1.74e-3,
                "shoulder" => 3.82e-3,
                "elbow" => 1.27e-3,
                "wrist" => 2.03e-5)

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