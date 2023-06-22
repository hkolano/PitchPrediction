% Outdated plotting function

elimd_gps = ["goal_poses", "manip_vels", "goal_vels", "xyz_poses", "manip_des_vels", "xyz_vels", "ry_vels", "manip_poses", "ry_poses"];

ablation_rmses = zeros(9, 4);
base_path = 'data\networks\full-nets\iros_nets';

this_level_rmses = zeros(1, 4);
for take_n = 1:3
    load(strcat(base_path, '0_nets\abl_0_no__take', string(take_n), '_droprate.mat'))
    this_level_rmses(take_n) = info.FinalValidationRMSE;
end
this_level_rmses(4) = mean(this_level_rmses(1:3));
ablation_rmses(1,:) = this_level_rmses;

for level = 1:8
    this_level_rmses = zeros(1, 4);
    for take_n = 1:4
        load(strcat(base_path, string(level), '_nets\abl_', string(level), '_no_', elimd_gps(level), '_take', string(take_n), '_droprate.mat'))
        this_level_rmses(take_n) = info.FinalValidationRMSE;
    end

    ablation_rmses(level+1,:) = this_level_rmses;
end

% Last level
this_level_rmses = zeros(1,4);
for take_n = 1:4
    load(strcat(base_path, string(level), '_nets\abl_', string(level), '_no_', elimd_gps(9), '_take', string(take_n), '_droprate.mat'))
    this_level_rmses(take_n) = info.FinalValidationRMSE;
end
ablation_rmses(10,:) = this_level_rmses;

disp(ablation_rmses);
output_file = fullfile("data\networks\full-nets", "ablation_rmses.mat");
save(output_file, 'ablation_rmses')