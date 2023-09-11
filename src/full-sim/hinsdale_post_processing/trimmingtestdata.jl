
all_traj_codes = ["003-0", "003-1", 
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
"030-0", "030-1"] #, 
# ["_alt_001-0", "_alt_001-1", "_alt_001-2", 
# "_alt_002-0", "_alt_002-1", 
# "_alt_008-0", 
# "_alt_008-1", "_alt_008-2", 
# "_alt_009-0", "_alt_009-1", 
# "_alt_011-0", "_alt_011-1", "_alt_011-2"]

for (i, trial_code) in enumerate(all_traj_codes)
    # get original IMU data
    imu_df = get_imu_data_from_csv(trial_code, "hinsdale-data-2023")
    imu_df = calc_rpy(imu_df)
    # get saved sim data (that includes downsampled imu)
    simdata_filepath = joinpath("data", "full-sim-data-091023", "test", trial_code*".csv")
    og_sim_df = CSV.read(simdata_filepath, DataFrame)

    imu_start = imu_df[1,:time_secs]
    imu_end = imu_df[end,:time_secs]
    println("At traj ID: $(trial_code)")
    println("IMU data starts at time $(imu_start) and ends at $(imu_end)")

    deleteat!(og_sim_df, findall(<(imu_start-.04), og_sim_df[!,:time_secs]))
    deleteat!(og_sim_df, findall(>(imu_end), og_sim_df[!,:time_secs]))
    # p = new_plot()
    # @df og_sim_df plot!(p, :time_secs, [:pitch, :qs2])
    # @df imu_df plot!(p, :time_secs, :pitch)
    # display(p)

    CSV.write(simdata_filepath, og_sim_df)
end