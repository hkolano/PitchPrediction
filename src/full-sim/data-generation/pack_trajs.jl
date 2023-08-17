path_to_data = joinpath("src", "full-sim", "data-generation", "combo_traj_yaml_files_otherhome")
traj_file_names = readdir(path_to_data)
traj_file_prefixes = [name[1:7] for name in traj_file_names]

series_names = unique(traj_file_prefixes)
for prefix in series_names 
    new_path = joinpath(path_to_data, prefix)
    if isdir(new_path) == false
        mkdir(new_path)
    end
end

for traj_file_name in traj_file_names
    # @show traj_file_name
    old_file_path = joinpath(path_to_data, traj_file_name)
    new_file_path = joinpath(path_to_data, traj_file_name[1:7], traj_file_name)
    # @show old_file_path
    # @show new_file_path
    mv(old_file_path, new_file_path)
end
# print(series_names)

# for file_name in traj_file_names
#     
# end