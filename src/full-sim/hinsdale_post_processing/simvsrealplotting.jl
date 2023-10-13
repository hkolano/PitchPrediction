using Plots, DataFrames

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

# Very good list: 
# 006-3 , 014-2, 024-2, alt 008-0

# medium good:
# alt 001-2

# EXCLUDE list 
# 012-3, 014-0, 014-1, 019-1, 020-0, alt 009-0, 009-1

# Oddities: 
# 003-1: hits the ground before starts
# 004-1: hits one joint limits, wobbles
# 005-1: doesn't track arm very well
# 005-2: hits joint limit and wobbles 
# 005-3: wobbles before and after (hits joint limit)
# 006-1: bad controller, doesn't hit end position
# 006-2: ?????
# 006-4, 007-0: bad controller, incorrect end position
# 012-0 shouldn't be as good as it is (speeds up in middle)
# 012-2 hits two joint limits
# 015-0; speeds up in middle 
# 015-1 hits joint limits
# 016-0 speeds up in middle, goes off track
# 016-1 coming off a wobble
# 020-1 hits joint limit
# 025-1 comes off wobble, hits 2joint limits
# alt 002-0 bad controller
# alt 002-1




# for name in all_traj_codes
#     make_plot_from_trial(name)
# end
sim_palette = palette([:deepskyblue2, :magenta], 4)
actual_palette = palette([:goldenrod1, :springgreen3], 4)

gr(size=(1200, 550))

function make_plot_from_trial(traj_name)
    combo_df = CSV.read(joinpath("data", "full-sim-data-091023", "test", traj_name*".csv"), DataFrame)

    p_js = new_plot()
    @df combo_df plot!(p_js, :time_secs, [:qs7.+3.07, :qs8, :qs9, :qs10.+2.879]; 
    palette=sim_palette, linewidth=2, linestyle=:dash, 
    label=["sim axis e" "sim axis d" "sim axis c" "sim axis b"])
    @df combo_df plot!(p_js, :time_secs, [:axis_e_pos, :axis_d_pos, :axis_c_pos, :axis_b_pos]; #, cols(11)]; 
    palette=:atlantic, linewidth=2, linestyle=:solid, 
    label=["act axis e" "act axis d" "act axis c" "act axis b"])
    plot!(legend=:outerright)

    p = new_plot()
    @df combo_df plot!(p, :time_secs, rad2deg.(:pitch), linewidth=2, linecolor=colorant"#3D65A5", label="IMU pitch")
    @df combo_df plot!(p, :time_secs, rad2deg.(:roll), linewidth=2, linecolor=colorant"#E57A77", label="IMU roll")
    @df combo_df plot!(p, :time_secs, rad2deg.(:qs2), linewidth=2, linestyle=:dash, linecolor=colorant"#1F449C", label="Sim pitch")
    @df combo_df plot!(p, :time_secs, rad2deg.(:qs1), linewidth=2, linestyle=:dash, linecolor=colorant"#F05039", label="Sim roll")
    
    # p_xyz = new_plot()
    # @df combo_df plot!(p_xyz, :time_secs, [:qs4, :qs5, :qs6])

    plot!(p, legend=false)
    plot!(p, titlefontsize = 16, labelfontsize=14, legend=:none)
    plot!(p, top_margin = 2mm, bottom_margin=5mm, right_margin=2mm, left_margin=0mm)

    # super_plot = plot(p_js, p, layout=(2,1), plot_title="Trial $(traj_name)")
    # display(super_plot)
    # ylabel!(p, "Vehicle Orientation (degrees)")
    return p
end

# Pretty good trajectories
p1 = make_plot_from_trial("014-2")
title!(p1, "Trial 24")
plot!(p1, left_margin=20mm)
# ylabel!(p1, "Vehicle Orientation (degrees)")

p2 = make_plot_from_trial("006-3")
title!(p2, "Trial 12", titlefont=Plots.font(16, "arial"))
# plot!(p2, left_margin=10mm)
# ylabel!(p2, "Vehicle Orientation (degrees)")

# joint limit hits 
p3 = make_plot_from_trial("005-2")
title!(p3, "Trial 7")

# speeds up/"catches up'
p4 = make_plot_from_trial("015-0")
title!(p4, "Trial 25")
# plot!(p4, legend=:outerbottom)

# hits bottom during trial
p5 = make_plot_from_trial("003-1")
title!(p5, "Trial 2")

# uses thrusters during trial
p6 = new_plot()
p6 = make_plot_from_trial("009-1")
title!(p6, "Trial 12")
# plot!(p6, legend=:outerbottom, legend_columns=4)


l = grid(2, 3)
super_plot = plot(p1, p3, p5, p2, p4, p6, layout=l) #, plot_title="Trial vs Simulation Vehicle Orientation", plot_title_font_size=16)
plot!(super_plot, xlabelfontsize=12, gridlinewidth=1.5) #, grid = (:x, :solid, .75, .9))
# ylabel!(super_plot, plot_ylabel="Vehicle Orientation (degrees)")
display(super_plot)

savefig("data/plots/IMUvsSim1200x550.png")