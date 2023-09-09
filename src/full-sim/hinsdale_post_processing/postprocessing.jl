using CSV
using DataFrames, StatsPlots
using Rotations

include("plotting.jl")

#%%

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
"030-0", "030-1", 
"_alt_001-0", "_alt_001-1", "_alt_001-2", 
"_alt_002-0", "_alt_002-1", 
"_alt_008-0", 
"_alt_008-1", "_alt_008-2", 
"_alt_009-0", "_alt_009-1", 
"_alt_011-0", "_alt_011-1", "_alt_011-2"]


# ----------------------------------------------------------------------------
#                  Trim start and end of rosbag data
# ----------------------------------------------------------------------------
perform_trimming = false

function trim_start(df, cutoff_time)
    for i in reverse(1:nrow(df))
        if df[i, 1] < cutoff_time 
            return df[i+1:end, :]
        end
    end
    throw(BoundsError())
end

function trim_end(df, cutoff_time)
    for i in 1:nrow(df)
        if df[i, 1] > cutoff_time
            return df[1:i, :]
        end
    end
    throw(BoundsError())
end          

if perform_trimming == true
    for trial_code in all_traj_codes

        println(trial_code)

        mocap_datapath = joinpath("data", "hinsdale-data-2023", "traj"*trial_code*"_mocap.csv")
        mocap_df = CSV.read(mocap_datapath, DataFrame)
        dropmissing!(mocap_df)

        js_filepath = joinpath("data", "hinsdale-data-2023", "traj"*trial_code*"_joint_states.csv")
        js_df = CSV.read(js_filepath, DataFrame)
        dropmissing!(js_df)

        imu_datapath = joinpath("data", "hinsdale-data-2023", "traj"*trial_code*"_imu.csv")
        imu_df = CSV.read(imu_datapath, DataFrame)

        p = plot_data(js_df, mocap_df)
        display(p)
        sleep(1)

        println("Please enter a cutoff time for this trajectory.")
        local user_cutoff
        try
            user_cutoff = parse(Float64, readline())
        catch e
            println(e)
        else
            mocap_df = trim_end(mocap_df, user_cutoff)
            js_df = trim_end(js_df, user_cutoff)
            imu_df = trim_end(imu_df, user_cutoff)

            p = plot_data(js_df, mocap_df)
            display(p)
            sleep(0.5)
        end

        println("Please enter a start time for this trajectory.")
        local user_cutoff2
        try 
            user_cutoff2 = parse(Float64, readline())
        catch e
            println(e) 
        else 
            mocap_df = trim_start(mocap_df, user_cutoff2)
            js_df = trim_start(js_df, user_cutoff2)
            imu_df = trim_start(imu_df, user_cutoff2)

            mocap_df[:,1] = mocap_df[:,1] .- user_cutoff2
            js_df[:,1] = js_df[:,1] .- user_cutoff2
            imu_df[:,1] = imu_df[:,1] .- user_cutoff2

            p = plot_data(js_df, mocap_df)
            display(p)
            sleep(0.5)   
        end

        # CSV.write(mocap_datapath, mocap_df)
        # CSV.write(js_filepath, js_df)
        # CSV.write(imu_datapath, imu_df)

    end
end

#%%
# ----------------------------------------------------------------------------
#                   Add roll-pitch-yaw data to all dataframes
# ----------------------------------------------------------------------------
translate_to_rpy = true

# all_traj_codes = ["baseline1", "baseline2", "baseline3"]
all_traj_codes = ["003-0"]

if translate_to_rpy == true
    for trial_code in all_traj_codes
        mocap_datapath = joinpath("data", "hinsdale-data-notimetrim", "traj"*trial_code*"_mocap.csv")
        # mocap_datapath = joinpath("data", "hinsdale-data-2023", trial_code*"_mocap.csv")
        mocap_df = CSV.read(mocap_datapath, DataFrame)
        dropmissing!(mocap_df)
        new_df = calc_rpy(mocap_df)
        CSV.write(mocap_datapath, new_df)
    end
end


# function calc_rpy_forwardframe(mocap_df)
#     roll_vec = Vector{Float64}(undef, 0)
#     pitch_vec = Vector{Float64}(undef, 0)
#     yaw_vec = Vector{Float64}(undef, 0)

#     rhf_to_backwards = RotMatrix(SMatrix{3,3}([-1. 0 0; 0 -1 0; 0 0 1]))

#     for i in 1:nrow(mocap_df)
#         r = QuatRotation([mocap_df[i, :w_ori], mocap_df[i, :x_ori], mocap_df[i, :y_ori], mocap_df[i, :z_ori]])
#         euler = RotXYZ(rhf_to_backwards*r)
#         # euler = RotXYZ(r)
#         push!(roll_vec, euler.theta1)
#         push!(pitch_vec, euler.theta2)
#         push!(yaw_vec, euler.theta3)
#     end

#     mocap_df[!, "roll_ff"] = roll_vec
#     mocap_df[!, "pitch_ff"] = pitch_vec
#     mocap_df[!, "yaw_ff"] = yaw_vec
#     return mocap_df
# end