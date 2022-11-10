% Load in the data
load("data/full-data-matlab/FullData_081022.mat")

%%
load('data/channel_dict.mat')
chan_idxs = rmfield(chan_idxs, 'pitch');
chan_idxs = rmfield(chan_idxs, 'dt');


chan_idxs = rmfield(chan_idxs, {'xyz_poses', 'xyz_vels', 'ry_vels', 'manip_vels', 'manip_des_vels', 'goal_poses'});
% Make a list of all channel indices
all_idxs = 1:1:41;

% Initialize constants
k = 25;
numUnits = 128;
pitch_idx = 23;

% Set up vectors to store loss values
all_losses = {};
all_RMSEs = {};
removed_features = {};
feature_group_list = {};

for level_num = 1:2
    fn = fieldnames(chan_idxs)
    feature_group_list{level_num} = fn;
    level_losses = [];
    level_RMSEs = [];

    % Iterate over all remaining features
    for feat_idx = 1:length(fn)
        % What feature name are we removing?
        feat_name = fn{feat_idx}
        % What channels does that feature group contain?
        feat_chan_idxs = chan_idxs.(feat_name);
    
        % Get the indices of the remaining channels, sans the feature group in
        % question
        rem_idxs = all_idxs(~ismember(all_idxs, feat_chan_idxs));
    
        % Cut the group's data out of the dataset
        clear Resp_Train Inputs_Train Resp_Test Inputs_Test
        [Inputs_Train, Resp_Train] = split_data(XTrain, pitch_idx, k, rem_idxs);
        [Inputs_Test, Resp_Test] = split_data(XTest, pitch_idx, k, rem_idxs);
    
        % How many feature channels remain?
        numChannels = size(Inputs_Train{1}, 1)
        
        % Initialize the neural network
        layers = setup_lookahead_rnn(numChannels, k, numUnits);
        init_options = define_new_opts(Inputs_Test, Resp_Test);
        subgroup_losses = [];
        subgroup_RMSEs = [];
    
       for take_n = 1:1 % 3
            [net, info] = trainNetwork(Inputs_Train,Resp_Train,layers,init_options);
            
            subgroup_losses = [subgroup_losses, info.FinalValidationLoss];
            subgroup_RMSEs = [subgroup_RMSEs, info.FinalValidationRMSE];
            %     
            outputFile = strcat("data/networks/icra-redo-nets/abl_rnd", string(level_num), "/no_", feat_name, '_take_', string(take_n), '.mat');
            save(outputFile, 'net', 'info');
       end
    
       level_losses = [level_losses, mean(subgroup_losses)];
       level_RMSEs = [level_RMSEs, mean(subgroup_RMSEs)];
    end

    % Record losses for this level
    all_losses{level_num} = level_losses;
    all_RMSEs{level_num} = level_RMSEs;
    disp(level_losses)

    % Determine which feature has the lowest loss
    [min_val, min_idx] = min(level_losses);
    smallest_impact_feat = fn{min_idx}
    feat_chans_to_remove = chan_idxs.(smallest_impact_feat);
    removed_features{level_num} = smallest_impact_feat

    % Remove the feature for the next round
    all_idxs = all_idxs(~ismember(all_idxs, feat_chans_to_remove));
    chan_idxs = rmfield(chan_idxs, smallest_impact_feat);
end

outputfile = strcat("data/networks/icra-redo-nets/ablationstudyresults.mat");
save(outputfile, 'all_losses', 'all_RMSEs', 'feature_group_list', 'removed_features');
% disp("Final Losses:")
% disp(all_losses)
% disp("Final RMSEs:")
% disp(all_RMSEs)
% beep

% end

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
        MaxEpochs = 1, ...
        MiniBatchSize=16, ...
        SequencePaddingDirection="right", ...
        Plots="training-progress", ...
        Shuffle='every-epoch', ...
        ValidationData={val_inputs, val_outputs}, ...
        ValidationFrequency = 60, ...
        OutputNetwork='best-validation-loss');
end


