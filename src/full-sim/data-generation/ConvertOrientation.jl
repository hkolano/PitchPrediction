using Rotations, StaticArrays, PitchPrediction, DataFrames, Tables

src_dir = dirname(pathof(PitchPrediction))
quat_file = joinpath(src_dir, "..", "data","full-sim-data", "data-quat", "quats1.csv")
f = CSV.read(quat_file, DataFrame)

angle_axis_array = Array{Float64}(undef, length(f.qs0), 4)
rpy_array = Array{Float64}(undef, length(f.qs0), 3)
fill!(angle_axis_array, 0.0)
fill!(rpy_array, 0.0)

n = 1
for row in CSV.Rows(quat_file, types=Float64)
    quat = QuatRotation([row.qs0, row.qs1, row.qs2, row.qs3])
    
    # aa = AngleAxis(quat)
    ϕ = rotation_angle(quat)
    v = rotation_axis(quat)
    angle_axis_array[n,:] = [ϕ, v...]

    euler = RotXYZ(quat)    
    rpy_array[n, :] = [euler.theta1, euler.theta2, euler.theta3] 

    global n += 1
end

aa_table = Tables.table(angle_axis_array)
aa_labels = ["Angle, Axis1, Axis2, Axis3"]
rpy_table = Tables.table(rpy_array)
rpy_labels = ["Roll", "Pitch", "Yaw"]
CSV.write("data/full-sim-data/data-angleaxis/aas1.csv", aa_table, header=aa_labels)
CSV.write("data/full-sim-data/data-rpy/rpys1.csv", rpy_table, header=rpy_labels)

println("Converted all orientations")