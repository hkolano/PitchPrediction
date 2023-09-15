
all_traj_codes =["003-0", "003-1", 
"004-0", "004-1", 
"005-0", "005-1", "005-2", "005-3", 
"006-0", "006-1", "006-2", "006-3", "006-4", 
"007-0", "007-1",
"009-0", "009-1",
"012-0", "012-1", "012-2", "012-3",
"014-0", "014-1", "014-2", 
"015-0", "015-1", 
"016-0", "016-1", 
"019-0", "019-1", "019-2", "019-3",
"020-0", "020-1", 
"024-0", "024-1", "024-2", 
"025-0", "025-1", 
"026-0", "026-1", "026-2", 
"030-0", "030-1", 
"_alt_001-0", "_alt_001-1", "_alt_001-2", 
"_alt_002-0", "_alt_002-1", 
"_alt_008-0", 
"_alt_008-1", "_alt_008-2", 
"_alt_009-0", "_alt_009-1", 
"_alt_011-0", "_alt_011-1", "_alt_011-2"]

raw_rmses_pitch = []
offset_rmses_pitch = []
raw_rmses_roll = []
offset_rmses_roll = []


# EXCLUDE list 
# 012-3, 014-0, 014-1, 019-1, 020-0, alt 009-0, 009-1
top_50_trajs =
["003-0", "003-1", 
"004-0", "004-1", 
"005-0", "005-1", "005-2", "005-3", 
"006-0", "006-1", "006-2", "006-3", "006-4", 
"007-0", "007-1",
"009-0", 
"012-0", "012-1", "012-2", 
"014-2", 
"015-0", "015-1", 
"016-0", "016-1", 
"019-0", "019-2", "019-3",
"020-1", 
"024-0", "024-1", "024-2", 
"025-0", "025-1", 
"026-0", "026-1", "026-2", 
"030-0", "030-1", 
"_alt_001-0", "_alt_001-1", "_alt_001-2", 
"_alt_002-0", "_alt_002-1", 
"_alt_008-0", "_alt_008-1", "_alt_008-2", 
"_alt_009-1", 
"_alt_011-0", "_alt_011-1", "_alt_011-2"]

a_offset = -1.8
num_offset_steps = -Int(a_offset/.04)

max_pitches = []
min_pitches = []

trial_code = "009-1"
# for (i, trial_code) in enumerate(top_50_trajs)
    # get original IMU data
    # imu_df = get_imu_data_from_csv(trial_code, "hinsdale-data-2023", true)
    # imu_df = calc_rpy(imu_df)
    mocap_df = get_vehicle_response_from_csv(trial_code, "hinsdale-data-2023", false)
    # get saved sim data (that includes downsampled imu)
    simdata_filepath = joinpath("data", "full-sim-data-091023", "test", trial_code*".csv")
    # og_sim_df = CSV.read(simdata_filepath, DataFrame)

    # ===== Retroactively adding angular velocities into the dataset =====
    # const_dt_imu_angvels_df = interp_at_timesteps(og_sim_df[!,:time_secs], imu_df, [:x_angvel, :y_angvel, :z_angvel], "linear")
    # og_sim_df = hcat(og_sim_df, const_dt_imu_angvels_df[!,[:x_angvel, :y_angvel, :z_angvel]])
    
    # ===== Print the time the imu data starts and ends =====
    # imu_start = imu_df[1,:time_secs]
    # imu_end = imu_df[end,:time_secs]
    # println("At traj ID: $(trial_code)")
    # println("IMU data starts at time $(imu_start) and ends at $(imu_end)")

    # ===== Get range of pitches in the trajectory =====
    # push!(max_pitches, maximum(og_sim_df[!,:qs2]))
    # push!(min_pitches, minimum(og_sim_df[!,:qs2]))

    # ===== Calculate the rmse between actual and simulated roll and pitch =====
    # raw_rmse_pitch = rmsd(og_sim_df[!,:qs2], og_sim_df[!,:pitch])
    # raw_rmse_roll = rmsd(og_sim_df[!,:qs1], og_sim_df[!,:roll])
    # offset_rmse_pitch = rmsd(og_sim_df[!,:qs2][num_offset_steps+1:end], og_sim_df[!,:pitch][1:end-num_offset_steps])
    # offset_rmse_roll = rmsd(og_sim_df[!,:qs1][num_offset_steps+1:end], og_sim_df[!,:roll][1:end-num_offset_steps])
    # push!(raw_rmses_pitch, rad2deg(raw_rmse_pitch))
    # push!(raw_rmses_roll, rad2deg(raw_rmse_roll))
    # push!(offset_rmses_pitch, rad2deg(offset_rmse_pitch))
    # push!(offset_rmses_roll, rad2deg(offset_rmse_roll))

    # ===== Delete beginning and end of trajectory that doesn't overlap with IMU data =====
    # deleteat!(og_sim_df, findall(<(imu_start-.04), og_sim_df[!,:time_secs]))
    # deleteat!(og_sim_df, findall(>(imu_end), og_sim_df[!,:time_secs]))

    # ===== Plot! =====
    # p = new_plot()
    # @df og_sim_df plot!(p, :time_secs, [:pitch, :qs2])
    # title!("Traj $(trial_code)")
    # @df imu_df plot!(p, :time_secs, :pitch)
    # @df og_sim_df plot!(p, :time_secs, [:qs7.+3.07, :qs8, :qs9, :qs10.+2.879])
    # @df og_sim_df plot!(p, :time_secs, [:axis_e_pos, :axis_d_pos, :axis_c_pos, :axis_b_pos])
    # plot!(p, y_lims=(-.5, 6))
    # display(p)

    # ===== Write the data back to the filepath =====
    # CSV.write(simdata_filepath, og_sim_df)
# end


# ----------------------------------------------------------
#             Show RMSE data for roll/pitch
# ----------------------------------------------------------
#%%
# @show mean(raw_rmses_pitch)
# @show std(raw_rmses_pitch, mean=mean(raw_rmses_pitch))
# @show mean(raw_rmses_roll)
# @show std(raw_rmses_roll, mean=mean(raw_rmses_roll))

# @show mean(offset_rmses_pitch)
# @show std(offset_rmses_pitch, mean=mean(offset_rmses_pitch))
# @show mean(offset_rmses_roll)
# @show std(offset_rmses_roll, mean=mean(offset_rmses_roll))