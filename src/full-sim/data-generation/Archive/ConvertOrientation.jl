#= 
Converts a csv of quaternions into a csv of rpy values. 

Deprecated in use as of March 2023. 
=# 
using Rotations, StaticArrays, DataFrames, Tables, CSV

convert_hydro = false
convert_single_folder = true

# ---------------------------------------------------------------------
#                           FUNCTIONS
# ---------------------------------------------------------------------
function convert_files_in_folder(dir_to_read, dir_to_write_str)
    files_to_read = readdir(dir_to_read)
    global num_files_converted = 0

    for file_name in files_to_read
        # Get its index
        file_digits = filter(isdigit, file_name)
        # println(file_digits)
        file_idx = parse(Int, file_digits)

        # Get the full file path
        quat_file = joinpath(dir_to_read, file_name)
        # Read this file to a dataframe (just to get its length)
        f = CSV.read(quat_file, DataFrame)

        # Create empty array
        rpy_array = Array{Float64}(undef, length(f.qs0), 3)
        fill!(rpy_array, 0.0)

        # Iterate through each row in the file
        m = 1
        for row in CSV.Rows(quat_file, types=Float64)
            quat = QuatRotation([row.qs0, row.qs1, row.qs2, row.qs3])

            # Convert to RPY representation
            euler = RotXYZ(quat)    
            rpy_array[m, :] = [euler.theta1, euler.theta2, euler.theta3] 

            m += 1
        end

        # Save rpys to new files
        rpy_table = Tables.table(rpy_array)
        rpy_labels = ["Roll", "Pitch", "Yaw"]
        CSV.write(joinpath(dir_to_read, "..", dir_to_write_str, "rpys$(file_idx).csv"), rpy_table, header=rpy_labels)

        global num_files_converted = num_files_converted + 1
        if rem(num_files_converted, 50) == 0
            println("$(num_files_converted) files converted.")
        end
    end
end

# ---------------------------------------------------------------------
#                    Single Folder Conversion
# ---------------------------------------------------------------------
if convert_single_folder == true
    # Assumes current folder is PitchPrediction
    data_folder = joinpath(pwd(), "data", "full-sim-data-110822", "data-quat")
    convert_files_in_folder(data_folder, "data-rpy")
end

# ---------------------------------------------------------------------
#                    Uncertain Hydro Conversion
# ---------------------------------------------------------------------
if convert_hydro == true
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

            convert_files_in_folder(quats_folder, "data-rpy-model$(model_num)")
        end
    end

    println("Converted all orientations")
end