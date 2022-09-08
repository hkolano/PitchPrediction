using Rotations, StaticArrays, PitchPrediction, DataFrames, Tables, CSV

param_names = ["arm-added-mass", "arm-linear-drag", "vehicle-added-mass", "vehicle-linear-drag", "vehicle-quadratic-drag"]

# Pick which directory to read from
src_dir = dirname(pathof(PitchPrediction))
data_folder = joinpath(src_dir, "..", "data", "full-sim-with-hydro")
subgroup_path = joinpath(data_folder, "single-model-50percent")

for param in param_names
    param_path = joinpath(subgroup_path, param)

    for model_num = 1:10
        file_name = "data-quat-model$(model_num)"
        quats_folder = joinpath(param_path,  file_name)
        # Generate list of files to parse
        files_to_read = readdir(quats_folder)

        global num_files_converted = 0

        # Iterate through each file in the folder
        for file_name in files_to_read
            # Get its index
            file_digits = filter(isdigit, file_name)
            # println(file_digits)
            file_idx = parse(Int, file_digits)

            # Get the full file path
            quat_file = joinpath(quats_folder, file_name)
            # Read this file to a dataframe (just to get its length)
            f = CSV.read(quat_file, DataFrame)

            # Create empty arrays for the different file representations 
            angle_axis_array = Array{Float64}(undef, length(f.qs0), 4)
            rpy_array = Array{Float64}(undef, length(f.qs0), 3)
            fill!(angle_axis_array, 0.0)
            fill!(rpy_array, 0.0)

            # Iterate through each row in the file
            m = 1
            for row in CSV.Rows(quat_file, types=Float64)
                quat = QuatRotation([row.qs0, row.qs1, row.qs2, row.qs3])
                
                # Convert to angle-axis representation
                ϕ = rotation_angle(quat)
                v = rotation_axis(quat)
                angle_axis_array[m,:] = [ϕ, v...]

                # Convert to RPY representation
                euler = RotXYZ(quat)    
                rpy_array[m, :] = [euler.theta1, euler.theta2, euler.theta3] 

                m += 1
            end

            # Save angles to new files
            aa_table = Tables.table(angle_axis_array)
            aa_labels = ["Angle, Axis1, Axis2, Axis3"]
            rpy_table = Tables.table(rpy_array)
            rpy_labels = ["Roll", "Pitch", "Yaw"]
            # CSV.write("data/full-sim-data/data-angleaxis/aas$(file_idx).csv", aa_table, header=aa_labels)
            CSV.write(joinpath(param_path, "data-rpy-model$(model_num)", "rpys$(file_idx).csv"), rpy_table, header=rpy_labels)

            global num_files_converted = num_files_converted + 1
            if rem(num_files_converted, 50) == 0
                println("$(num_files_converted) files converted.")
            end
        end
    end
end

println("Converted all orientations")