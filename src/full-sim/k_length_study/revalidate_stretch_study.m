%{
Once k_length study has been run to produce the pitch-only prediction
networks, this script validates the networks on the specified indices and
trajectory locations in the validation set. 

Last modified 2/27/22
%}

load("data/full-sim-data-022223/FullData_50Hz.mat")
[sorted_XTest_50hz, ~] = sort_data_by_length(XTest);
val_set = 'data/full-sim-data-022223/val_set.mat';
load(val_set)

%%
load('data/full-sim-data-022223/channel_dict.mat')

pitch_idx = chan_idxs.act_pitch;
elimd_gps = ["meas_xyz", "meas_joint_vels", "meas_linear_vels"];
all_idxs = get_remaining_idxs(elimd_gps, chan_idxs);
all_idxs = [20, all_idxs]
%%

k = 25;
stretches = 1:8;
stretch_forecast_errors = zeros(3,length(stretches));

for idx = 1:length(stretches)
    sf = stretches(idx)
    [Inputs_Test, Resp_Test] = transform_data_for_stretch_study(sorted_XTest_50hz, sf, k, all_idxs, pitch_idx);

    for take_n = 1:3
        load(strcat("/nfs/stak/users/rosettem/PitchPrediction/data/networks/iros-nets/simple_w_stretch_factor/stretch_", string(sf), "_take_", string(take_n), ".mat"))
%         load(strcat("data\networks\iros-nets\simple_w_stretch_factor\stretch_", string(sf), "_take_", string(take_n), ".mat"))
        pitch_error = validate_pitch_on_stretch_forecast(net, Inputs_Test, Resp_Test, val_ns_50hz, val_idxs)
        stretch_forecast_errors(take_n, idx) = pitch_error;
    end
end

outputFile = fullfile("data/networks/iros-nets", 'k_study_results2.mat');
save(outputFile, 'stretch_forecast_errors');

function pitch_error = validate_pitch_on_stretch_forecast(net, XTest, G_truth, ns, val_idxs)
    error = 0;
    num_long_enough_trajs = 0;
    for i = 1:250
        if ns(i) > 125
            resetState(net);
            
            % Get values for this validation point
            pred_start_idx = ns(i);
            data = XTest{val_idxs(i)};
            traj_len = size(data, 2);
    %         fprintf('Trajectory length: %d ; prediction start %d \n', traj_len, pred_start_idx);
            
            [net, pred] = predictAndUpdateState(net, data(:,1:pred_start_idx));
            pred_forecast = pred(:, end);
            g_truth = G_truth{val_idxs(i)}(:,pred_start_idx);
            %rmse = sqrt(immse(pred_forecast, single(g_truth)));
%             disp("new traj")
%             disp([pred_forecast single(g_truth)])
            RMSE = rmse(pred_forecast, single(g_truth));
            error = error + RMSE;
            num_long_enough_trajs = num_long_enough_trajs + 1;
        end
    end
    pitch_error = error/num_long_enough_trajs;
end