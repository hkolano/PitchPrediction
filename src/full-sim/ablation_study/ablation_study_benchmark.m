%{
Trains a neural network on all 24 inputs 3 times. Run either before or
after the ablation study. 

Make sure data input is the correct one, and that MaxEpochs is set to 50 in
define_new_opts. 

Last modified 3/8/23
%}
% Load in the data
load("data/full-sim-data-022223/FullData_50Hz.mat")

%%
load('data/full-sim-data-022223/channel_dict.mat')
pitch_idx = chan_idxs.act_pitch;
chan_idxs = rmfield(chan_idxs, {'act_rpy', 'act_xyz', 'act_joint_pos', 'act_angular_vels', 'act_linear_vels', 'act_joint_vels', 'act_pitch'});

% Make a list of all measured indices
all_idxs = 21:1:44;

% Initialize constants
k = 25;
numUnits = 128;

% Set up vectors to store loss values
all_losses = {};
all_RMSEs = {};
removed_features = {};
feature_group_list = {};

level_losses = [];
level_RMSEs = [];

% Cut the group's data out of the dataset
clear Resp_Train Inputs_Train Resp_Test Inputs_Test
[Inputs_Train, Resp_Train] = split_data(XTrain, pitch_idx, k, all_idxs);
[Inputs_Test, Resp_Test] = split_data(XTest, pitch_idx, k, all_idxs);

% How many feature channels remain?
numChannels = size(Inputs_Train{1}, 1)

% Initialize the neural network
layers = setup_lookahead_rnn(numChannels, k, numUnits);
init_options = define_new_opts(Inputs_Test, Resp_Test);
subgroup_losses = [];
subgroup_RMSEs = [];

level_num = 0;
for take_n = 1:3
    [net, info] = trainNetwork(Inputs_Train,Resp_Train,layers,init_options);
    
    subgroup_losses = [subgroup_losses, info.FinalValidationLoss];
    subgroup_RMSEs = [subgroup_RMSEs, info.FinalValidationRMSE];
    %     
    outputFile = strcat("data/networks/iros-nets/abl_rnd", string(level_num), '/baseline_take_', string(take_n), '.mat');
    save(outputFile, 'net', 'info');
end

level_losses = [level_losses, mean(subgroup_losses)];
level_RMSEs = [level_RMSEs, mean(subgroup_RMSEs)];
%     end

    % Record losses for this level
%     all_losses{level_num} = level_losses;
%     all_RMSEs{level_num} = level_RMSEs;
    disp(level_losses)

    % Determine which feature has the lowest loss
%     [min_val, min_idx] = min(level_losses);
%     smallest_impact_feat = fn{min_idx}
%     feat_chans_to_remove = chan_idxs.(smallest_impact_feat);
%     removed_features{level_num} = smallest_impact_feat

    % Remove the feature for the next round
%     all_idxs = all_idxs(~ismember(all_idxs, feat_chans_to_remove));
%     chan_idxs = rmfield(chan_idxs, smallest_impact_feat);
% end

outputfile = strcat("data/networks/icra-redo-nets/ablationstudy_baselineresults.mat");
save(outputfile, 'level_losses', 'level_RMSEs');
% disp("Final Losses:")
% disp(all_losses)
% disp("Final RMSEs:")
% disp(all_RMSEs)
% beep

% To close all plots:
% delete(findall(0));

% ------------------------------------------------------------------------
%                                Functions
% ------------------------------------------------------------------------

function [inputs, outputs] = split_data(data, pitch_idx, k, rem_feat_idxs)
    for n = 1:numel(data)
        resp = zeros(k, size(data{n}, 2)-k);
        if size(data{n}, 2) > 1.1*k
            for t = 1:size(data{n}, 2)-k
                resp(:,t) = data{n}(pitch_idx, t+1:t+k)';
            end
            outputs{n} = resp;
            inputs{n} = data{n}(rem_feat_idxs,1:end-k);
        end
    end
end

function init_options = define_new_opts(val_inputs, val_outputs)
    init_options = trainingOptions("adam", ...
        InitialLearnRate=0.001,...
        LearnRateDropPeriod=5, ...
        LearnRateSchedule='piecewise', ...
        LearnRateDropFactor=.8, ...
        MaxEpochs = 50, ...
        MiniBatchSize=16, ...
        SequencePaddingDirection="right", ...
        Plots="none", ...
        Shuffle='every-epoch', ...
        ValidationData={val_inputs, val_outputs}, ...
        ValidationFrequency = 60, ...
        OutputNetwork='best-validation-loss');
end



