using CSV
using DataFrames, StatsPlots

#%%
gr(size=(800,600)) 

trial_code = "030-0"
# trial_code = "baseline1"

js_filepath = joinpath("data", "hinsdale-data-2023", "traj"*trial_code*"_joint_states.csv")
# js_filepath = joinpath("data", "hinsdale-data-2023", trial_code*"_joint_states.csv")
js_df = CSV.read(js_filepath, DataFrame)
dropmissing!(js_df)

imu_datapath = joinpath("data", "hinsdale-data-2023", "traj"*trial_code*"_imu.csv")
# imu_datapath = joinpath("data", "hinsdale-data-2023", trial_code*"_imu.csv")
imu_df = CSV.read(imu_datapath, DataFrame)

mocap_datapath = joinpath("data", "hinsdale-data-2023", "traj"*trial_code*"_mocap.csv")
# mocap_datapath = joinpath("data", "hinsdale-data-2023", trial_code*"_mocap.csv")
mocap_df = CSV.read(mocap_datapath, DataFrame)
dropmissing!(mocap_df)

function new_plot()
    plot(xlabel = "Time (s)")
end 

# plot joint states

function plot_data(js_df, mocap_df)
    p_js = new_plot()
    @df js_df plot!(p_js, :time_secs, cols(3:6))
    xaxis!(p_js, grid = (:x, :solid, .75, 0.9), minorgrid = (:x, :dot, 0.5, .5))
    ylabel!("Joint position (rad)")

    # plot vehicle pose 
    p_mocap = new_plot()
    @df mocap_df plot!(p_mocap, :time_secs, cols(9:10))
    plot!(p_mocap, label=names(mocap_df)[9:10])
    ylabel!("Radians")
    xaxis!(p_mocap, grid = (:x, :solid, .75, 0.9), minorgrid = (:x, :dot, 0.5, .5))
    # xgrid!(p_mocap, true)

    p_pose = new_plot()
    @df mocap_df plot!(p_pose, :time_secs, cols(2:4))
    plot!(p_pose, label=names(mocap_df)[2:4])
    ylabel!("Position (meters)")
    xaxis!(p_pose, grid = (:x, :solid, .75, 0.9), minorgrid = (:x, :dot, 0.5, .5))
    # display(p_mocap)

    super_plot = plot(p_js, p_mocap, p_pose, layout = (3, 1))
    return super_plot
end