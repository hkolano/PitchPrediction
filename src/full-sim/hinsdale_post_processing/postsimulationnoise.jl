using Glob, DataFrames

# Get all trajectories in a folder
traj_file_names = glob("*.csv", joinpath("data", "full-sim-data-091023", "train"))

# Set up noise distributions
v_ang_vel_noise_dist = Distributions.Normal(0, .0013) # 75 mdps (LSM6DSOX)
q_ang_noise_dist = Distributions.Normal(0, .0017) # .1 degrees, WAG (yaw drifts 0.1-0.3 deg/minute)
arm_pos_noise_dist = Distributions.Normal(0, .0017) # .1 degrees, from Reach website
arm_vel_noise_dist = Distributions.Normal(0, .0017*2) # twice the noise from the positions

v_ang_vel_extra_noise_dist = Distributions.Normal(0, .0013*5) # 75 mdps (LSM6DSOX)
q_ang_extra_noise_dist = Distributions.Normal(0, .0017*5) # .1 degrees, WAG (yaw drifts 0.1-0.3 deg/minute)
arm_pos_extra_noise_dist = Distributions.Normal(0, .0017*5) # .1 degrees, from Reach website
arm_vel_extra_noise_dist = Distributions.Normal(0, .0017*10)

# file_name = "data/full-sim-data-091023/train/0006.csv"
# Iterate through each file

for file_name in traj_file_names[]

    # read file into dataframe
    file_df = CSV.read(file_name, DataFrame)
    traj_length = nrow(file_df)

    file_df[!,:qs7adj] = file_df[!,:qs7] .+ 3.07
    file_df[!,:qs10adj] = file_df[!,:qs10] .+ 2.879

    # Add noise to angular velocities
    file_df[!,:noisy_vs1] = file_df[!,:vs1] + rand(v_ang_vel_noise_dist, traj_length)
    file_df[!,:noisy_vs2] = file_df[!,:vs2] + rand(v_ang_vel_noise_dist, traj_length)
    file_df[!,:noisy_vs3] = file_df[!,:vs3] + rand(v_ang_vel_noise_dist, traj_length)

    # add noise to orientations
    file_df[!,:noisy_qs1] = file_df[!,:qs1] + rand(q_ang_noise_dist, traj_length)
    file_df[!,:noisy_qs2] = file_df[!,:qs2] + rand(q_ang_noise_dist, traj_length)

    # add noise to joint states 
    file_df[!,:noisy_qs7] = file_df[!,:qs7adj] + rand(arm_pos_noise_dist, traj_length)
    file_df[!,:noisy_qs8] = file_df[!,:qs8] + rand(arm_pos_noise_dist, traj_length)
    file_df[!,:noisy_qs9] = file_df[!,:qs9] + rand(arm_pos_noise_dist, traj_length)
    file_df[!,:noisy_qs10] = file_df[!,:qs10adj] + rand(arm_pos_noise_dist, traj_length)

    # add noise to joint velocities
    file_df[!,:noisy_vs7] = file_df[!,:vs7] + rand(arm_vel_noise_dist, traj_length)
    file_df[!,:noisy_vs8] = file_df[!,:vs8] + rand(arm_vel_noise_dist, traj_length)
    file_df[!,:noisy_vs9] = file_df[!,:vs9] + rand(arm_vel_noise_dist, traj_length)
    file_df[!,:noisy_vs10] = file_df[!,:vs10] + rand(arm_vel_noise_dist, traj_length)

    # EXTRA noise to angular velocities
    file_df[!,:extranoisy_vs1] = file_df[!,:vs1] + rand(v_ang_vel_extra_noise_dist, traj_length)
    file_df[!,:extranoisy_vs2] = file_df[!,:vs2] + rand(v_ang_vel_extra_noise_dist, traj_length)
    file_df[!,:extranoisy_vs3] = file_df[!,:vs3] + rand(v_ang_vel_extra_noise_dist, traj_length)

    # EXTRA noise to orientations
    file_df[!,:extranoisy_qs1] = file_df[!,:qs1] + rand(q_ang_extra_noise_dist, traj_length)
    file_df[!,:extranoisy_qs2] = file_df[!,:qs2] + rand(q_ang_extra_noise_dist, traj_length)

    # EXTRA noise to joint states 
    file_df[!,:extranoisy_qs7] = file_df[!,:qs7adj] + rand(arm_pos_extra_noise_dist, traj_length)
    file_df[!,:extranoisy_qs8] = file_df[!,:qs8] + rand(arm_pos_extra_noise_dist, traj_length)
    file_df[!,:extranoisy_qs9] = file_df[!,:qs9] + rand(arm_pos_extra_noise_dist, traj_length)
    file_df[!,:extranoisy_qs10] = file_df[!,:qs10adj] + rand(arm_pos_extra_noise_dist, traj_length)

    # EXTRA noise to joint velocities
    file_df[!,:extranoisy_vs7] = file_df[!,:vs7] + rand(arm_vel_extra_noise_dist, traj_length)
    file_df[!,:extranoisy_vs8] = file_df[!,:vs8] + rand(arm_vel_extra_noise_dist, traj_length)
    file_df[!,:extranoisy_vs9] = file_df[!,:vs9] + rand(arm_vel_extra_noise_dist, traj_length)
    file_df[!,:extranoisy_vs10] = file_df[!,:vs10] + rand(arm_vel_extra_noise_dist, traj_length)
 
    select!(file_df, Not([:qs4, :qs5, :qs6, :qs7, :qs10, :vs4, :vs5, :vs6, :w_ori, :x_ori, :y_ori, :z_ori]))
    # CSV.write(file_name, file_df)
end