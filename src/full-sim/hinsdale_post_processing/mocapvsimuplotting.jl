gr(size=(800, 450))
imu_df = get_imu_data_from_csv("003-0", "hinsdale-data-2023")
imu_df = calc_rpy(imu_df)

mocap_df = get_vehicle_response_from_csv("003-0", "hinsdale-data-2023")

new_plot()
@df mocap_df plot!(:time_secs, :pitch, linewidth=2, linecolor=colorant"#3D65A5", label="Mocap pitch")
@df mocap_df plot!(:time_secs, :roll, linewidth=2, linecolor=colorant"#E57A77", label="MoCap roll")
@df imu_df plot!(:time_secs, :pitch, linewidth=2, linestyle=:dash, linecolor=colorant"#1F449C", label="IMU pitch")
@df imu_df plot!(:time_secs, :roll, linewidth=2, linestyle=:dash, linecolor=colorant"#F05039", label="IMU roll")

plot!(titlefontsize = 18)
plot!(top_margin = 2mm, bottom_margin=5mm, right_margin=2mm, left_margin=5mm)

ylabel!("Vehicle Orientation (rad)")
title!("BlueROV Orientation Measurement Comparison")

savefig("data/plots/MocapvsIMU_800by450.png")