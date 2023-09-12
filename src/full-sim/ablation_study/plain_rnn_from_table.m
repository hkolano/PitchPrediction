%{
Trains a neural network on training data from Julia, all saved as tables. 

Make sure data input is the correct one, and that MaxEpochs is set to 50 in
define_new_opts. 

Last modified 3/8/23
%}
% Load in the data
load("data/full-sim-data-091023/NormalizedTableTrainData.mat")

des_pred_length = 2.; % seconds
sim_dt = sequence_data{1}{2,"time_secs"} - sequence_data{1}{1, "time_secs"};
des_pred_steps = round(des_pred_length/sim_dt);
gt_pitch_column_name = "qs2";
gt_roll_column_name = "qs1";

gt_data = {};

sequence_data_array = {};
gt_array = {};


%%
for n = 1:numel(sequence_data)
    sequence_data{n} = sequence_data{n}(:,["qs1", "qs2", "qs3", "qs7", "qs8", "qs9", "qs10", "vs1", "vs2", "vs3"]);
    gt_data{n} = array2table(zeros(height(sequence_data{n})-des_pred_steps, des_pred_steps*2));
    sequence_length = height(sequence_data{n})-des_pred_steps;
    for i = 1:sequence_length
        gt_data{n}{i,1:des_pred_steps} = sequence_data{n}{i+1:i+des_pred_steps, gt_pitch_column_name}';
        gt_data{n}{i,des_pred_steps+1:end} = sequence_data{n}{i+1:i+des_pred_steps, gt_roll_column_name}';
    end
    sequence_data_array{n} = table2array(sequence_data{n}(1:sequence_length,:))';
    gt_array{n} = table2array(gt_data{n})';
end

%%
numUnits = 384;

% Set up vectors to store loss values
all_losses = {};
all_RMSEs = {};
level_losses = [];
level_RMSEs = [];

% Cut the group's data out of the dataset
% clear Resp_Train Inputs_Train Resp_Test Inputs_Test
% [Inputs_Train, Resp_Train] = split_data(XTrain, pitch_idx, k, all_idxs);
% [Inputs_Test, Resp_Test] = split_data(XTest, pitch_idx, k, all_idxs);

% How many feature channels remain?
numChannels = width(sequence_data{1});

% Initialize the neural network
layers = setup_lookahead_rnn(numChannels, des_pred_steps*2, numUnits);
init_options = define_new_opts("takeme", "out");

level_num = 0;
% for take_n = 1:3
    [net, info] = trainNetwork(sequence_data_array',gt_array',layers,init_options);
    
    loss = info.FinalValidationLoss;
    RMSEs = info.FinalValidationRMSE;
    %     
    outputFile = strcat("data/networks/posthinsdalenets/rnn.mat");
    save(outputFile, 'net', 'info');
% end

level_losses = [level_losses, mean(subgroup_losses)];
level_RMSEs = [level_RMSEs, mean(subgroup_RMSEs)];
%     end

disp(level_losses)


% outputfile = strcat("data/networks/icra-redo-nets/ablationstudy_baselineresults.mat");
% save(outputfile, 'level_losses', 'level_RMSEs');


% To close all plots:
% delete(findall(0));

% ------------------------------------------------------------------------
%                                Functions
% ------------------------------------------------------------------------
%%
% function [inputs, outputs] = split_data(data, pitch_idx, k, rem_feat_idxs)
%     for n = 1:numel(data)
%         resp = zeros(k, size(data{n}, 2)-k);
%         if size(data{n}, 2) > 1.1*k
%             for t = 1:size(data{n}, 2)-k
%                 resp(:,t) = data{n}(pitch_idx, t+1:t+k)';
%             end
%             outputs{n} = resp;
%             inputs{n} = data{n}(rem_feat_idxs,1:end-k);
%         end
%     end
% end
% 
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
        ExecutionEnvironment='gpu');
end

        % ValidationData={val_inputs, val_outputs}, 
        % ValidationFrequency = 60, ...
                % OutputNetwork='best-validation-loss'


