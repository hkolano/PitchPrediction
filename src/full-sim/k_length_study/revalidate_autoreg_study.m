%{
Validates the autoregressive networks on the validation set, which includes
a set of 250 trajectories and an index to begin predicting from.

Last modified March 2023
%}
% Validation Set
load("data/full-sim-data-022223/FullData_50Hz.mat")
[sorted_XTest_50Hz, I] = sort_data_by_length(XTest);
load("data/full-sim-data-022223/FullData_10Hz.mat")
sorted_XTest_10hz = XTest(flip(I, 2));

val_set = 'data/full-sim-data-022223/val_set.mat';
load(val_set)

%%
load('data/full-sim-data-022223/channel_dict.mat')

pitch_idx = chan_idxs.act_pitch;
elimd_gps = ["meas_xyz", "meas_joint_vels", "meas_linear_vels"];
all_idxs = get_remaining_idxs(elimd_gps, chan_idxs);

% [sorted_XTest_50hz, I] = sort_data_by_length(XTest);
% sorted_XTest = XTest_10hz(flip(I,2));

ks = [5 10 20 30 40];
% pitch_idx_after_feature_removal = 2;
auto_forecast_errors = zeros(1, length(ks));
val_XTest_10hz = sorted_XTest_10hz(val_idxs)

for k_idx = 1:length(ks)
    k = ks(k_idx)
    load(strcat("/nfs/stak/users/rosettem/PitchPrediction/data/networks/iros-nets/consolidated_autoreg/k", string(k), "/k", string(k), "_take1_50epochs.mat"))
    error = validate_pitch_on_forecast_only(net, val_XTest_10hz, val_ns_10hz, k, all_idxs, pitch_idx)
    auto_forecast_errors(k_idx) = error;
end

outputFile = fullfile("data/networks/iros-nets", 'auto_study_results.mat');
save(outputFile, 'auto_forecast_errors');