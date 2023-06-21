val_set = 'data/full-sim-data-110822/val_set.mat';
load(val_set);

load('data/full-sim-data-110822/FullData.mat')
load('data/full-sim-data-110822/FullData_10Hz.mat')

[sorted_XTest_50hz, I] = sort_data_by_length(XTest);
sorted_XTest = XTest_10hz(flip(I,2));

elimd_gps = ["xyz_poses", "xyz_vels", "goal_poses", "manip_des_vels", "goal_vels"];
all_idxs = get_remaining_idxs(elimd_gps);

for n = 1:numel(sorted_XTest)
    sorted_Input_Test{n} = sorted_XTest{n}(all_idxs, :);
end

%%
ks = [5 10 20 30 40];
pitch_idx_after_feature_removal = 13;
auto_forecast_errors = zeros(1, length(ks));

for k_idx = 1:length(ks)
    k = ks(k_idx)
    load(strcat("data\networks\icra-redo-nets\10Hz_k", string(k), "\take1_50epochs.mat"))
    error = validate_pitch_on_forecast_only(net, sorted_Input_Test(val_idxs), val_ns_10hz, k, pitch_idx_after_feature_removal)
    auto_forecast_errors(k_idx) = error;
end

outputFile = fullfile("data/networks/icra-redo-nets", 'auto_study_results.mat');
save(outputFile, 'auto_forecast_errors');