sim_df = CSV.read(joinpath("data", "full-sim-data-091023", "train", "0008.csv"), DataFrame)
#%%
snap_time = 11
snap_steps = Int(floor(snap_time/.04))

snap_ypr = Vector(sim_df[snap_steps, [:qs3, :qs2, :qs1]])
snap_ori = RotZYX(snap_ypr...)
snap_quat = QuatRotation(snap_ori)

zero!(state)
set_configuration!(mvis, joint_dict["vehicle"], [snap_quat.w, snap_quat.x, snap_quat.y, snap_quat.z, 0., 0., 0.])
# set_configuration!(state, joint_dict["vehicle"], [.99956, .01133, -.0269, -.004, 0., 0., 0])
set_configuration!(mvis, joint_dict["base"], sim_df[snap_steps, :qs7adj]-2.879)
set_configuration!(mvis, joint_dict["shoulder"], sim_df[snap_steps, :qs8])
set_configuration!(mvis, joint_dict["elbow"], sim_df[snap_steps, :qs9])
set_configuration!(mvis, joint_dict["wrist"], sim_df[snap_steps, :qs10adj]-3.07)
#%%
gr(size=(850,475))
p_js = new_plot()
@df sim_df plot!(p_js, :time_secs, :qs7adj, linecolor=colorant"#332288", linewidth=3, label="axis e pos")
@df sim_df plot!(p_js, :time_secs, :qs8, linecolor=colorant"#44AA99", linewidth=3, label="axis d pos")
@df sim_df plot!(p_js, :time_secs, :qs9, linecolor=colorant"#DDCC77", linewidth=3, label="axis c pos")
@df sim_df plot!(p_js, :time_secs, :qs10adj, linecolor=colorant"#CC6677", linewidth=3, label="axis b pos")
vline!([1, 5, 8, 10, 18], linecolor=:black, linestyle=:dash, linewidth=3.5, label="")
ylabel!("Joint Position (rad)")
plot!(p_js, left_margin=6mm, top_margin=5mm, minorgrid = (:x, :dot, .5, .5), legend=:outerbottom, legend_columns=4)
savefig("data/plots/trajsnapsrollpitch.png")


p_ori = new_plot()
@df sim_df plot!(p_ori, :time_secs, rad2deg.(:qs1); linecolor=colorant"#E57A77", linewidth=3, label="roll")
@df sim_df plot!(p_ori, :time_secs, rad2deg.(:qs2); linecolor=colorant"#3D65A5", linewidth=3, label="pitch")
vline!([1, 5, 8, 10, 18], linecolor=:black, linestyle=:dash, linewidth=3.5, label="")
ylabel!("Vehicle Orienation (deg)")

plot!(p_ori, left_margin=6mm, top_margin=5mm, minorgrid = (:x, :dot, .5, .5), legend=:outerbottom, legend_columns=2)
savefig("data/plots/trajsnapsjointplot.png")