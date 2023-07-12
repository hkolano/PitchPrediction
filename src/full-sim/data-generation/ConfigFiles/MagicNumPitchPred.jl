# ----------------------------------------------------------
# mechanism information
# ----------------------------------------------------------
is_jaw_fixed = true
body_names = ["vehicle"]
dof_names = ["roll", "pitch", "yaw", "x", "y", "z"]
include("../CtlrParFiles/IrosActuatorLimits.jl")

# ----------------------------------------------------------
# Simulation information
# ----------------------------------------------------------
do_scale_traj = true        # Scale the trajectory? 
max_traj_scaling = 2        # maximum factor to scale trajectory duration by
duration_after_traj = 1.0   # How long to simulate after trajectory has ended

# ----------------------------------------------------------
# Controller information
# ----------------------------------------------------------
add_noise = true
include("../CtlrParFiles/IrosPitchPredPars.jl")
