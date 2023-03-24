# Actuator upper torque limits 
torque_lim_dict = Dict("roll" => 0.0, 
                        "pitch" => 0.0,
                        "yaw" => 20., 
                        "x" => 71.5,
                        "y" => 88.2,
                        "z" => 177.,
                        "base" => 10.,
                        "shoulder" => 10.,
                        "elbow" => 10.,
                        "wrist" => 0.6)

# D_tau limits for a controller at 100Hz
thruster_dtau_lim = 0.001
joint_dtau_lim = 0.1
dtau_lim_dict = Dict("roll" => 0.0, 
                    "pitch" => 0.0,
                    "yaw" => thruster_dtau_lim, 
                    "x" => thruster_dtau_lim,
                    "y" => thruster_dtau_lim,
                    "z" => thruster_dtau_lim,
                    "base" => joint_dtau_lim,
                    "shoulder" => joint_dtau_lim,
                    "elbow" => joint_dtau_lim,
                    "wrist" => .006, 
                    "jaw" => joint_dtau_lim)