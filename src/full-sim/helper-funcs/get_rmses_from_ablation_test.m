
elimd_gps = ["goal_poses", "manip_vels", "goal_vels", "xyz_poses", "manip_des_vels", "xyz_vels", "ry_vels", "manip_poses", "ry_poses"];

ablation_rmses = zeros(9, 3);
base_path = 'C:\Users\hkola\Stuff\_ONR\RNNs for pitch prediction\PitchPrediction\data\networks\full-nets\ablTake2_';

for level = 1:8
    this_level_rmses = zeros(1, 3);
    for take_n = 1:3
        load(strcat(base_path, string(level), '_nets\abl_', string(level), '_no_', elimd_gps(level), '_take', string(take_n), '_droprate.mat'))
        this_level_rmses(take_n) = info.FinalValidationRMSE;
    end

    ablation_rmses(level,:) = this_level_rmses;
end

% Last level
for take_n = 1:3
    load(strcat(base_path, string(level), '_nets\abl_', string(level), '_no_', elimd_gps(9), '_take', string(take_n), '_droprate.mat'))
    this_level_rmses(take_n) = info.FinalValidationRMSE;
end
ablation_rmses(9,:) = this_level_rmses;

disp(ablation_rmses);
output_file = fullfile("C:\Users\hkola\Stuff\_ONR\RNNs for pitch prediction\PitchPrediction\data\networks\full-nets", "ablation_rmses.mat");
save(output_file, 'ablation_rmses')