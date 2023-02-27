%% Setup
% Load data set
load("data/full-sim-data-022223/FullData_50Hz.mat")

% get indices of remaining feature groups
load('data/full-sim-data-022223/channel_dict.mat')
pitch_idx = chan_idxs.act_pitch;
chan_idxs = rmfield(chan_idxs, {'act_rpy', 'act_xyz', 'act_joint_pos', 'act_angular_vels', 'act_linear_vels', 'act_joint_vels', 'act_pitch'});

% Make a list of all channel indices
all_idxs = 21:1:44;
elimd_gps = ["meas_linear_vels"];
all_idxs = get_remaining_idxs(elimd_gps, chan_idxs);

%%
% Initialize constants
% ks =[5, 10, 15, 20, 25, 30, 40, 50, 60, 70, 75, 80, 90, 100, 125, 150, 175, 200];
k = 25;
numUnits = 384;
stretches = [1, 2, 3, 4, 5, 6, 7, 8];

all_losses = [];
subgroup_losses = [];
all_RMSEs = [];
subgroup_RMSEs = [];

for idx = 1:length(stretches)
%% Input/Responses setup
    clear Resp_Train Inputs_Train Resp_Test Inputs_Test

    sf = stretches(idx);
    subgroup_losses = [];
    subgroup_RMSEs = [];

    [Inputs_Train, Resp_Train] = transform_data_for_stretch_study(XTrain, sf, k, all_idxs, pitch_idx);
    
    [Inputs_Test, Resp_Test] = transform_data_for_stretch_study(XTest, sf, k, all_idxs, pitch_idx);

    numChannels = size(Inputs_Train{1}, 1);
    layers = setup_lookahead_rnn(numChannels, k, numUnits);
    
    init_options = trainingOptions("adam", ...
        InitialLearnRate=0.001,...
        LearnRateDropPeriod=5, ...
        LearnRateSchedule='piecewise', ...
        LearnRateDropFactor=.9, ...
        MaxEpochs = 1, ...
        MiniBatchSize=16, ...
        SequencePaddingDirection="right", ...
        Plots="none", ...
        Shuffle='every-epoch', ...
        ValidationData={Inputs_Test', Resp_Test'}, ...
        ValidationFrequency = 60, ...
        OutputNetwork='best-validation-loss');

    %% Train the net
   for take_n = 1:3
        [net, info] = trainNetwork(Inputs_Train,Resp_Train,layers,init_options);
        
        subgroup_losses = [subgroup_losses, info.FinalValidationLoss];
        subgroup_RMSEs = [subgroup_RMSEs, info.FinalValidationRMSE];
        %     
        outputFile = fullfile("data/networks/iros-nets/simple_w_stretch_factor", strcat('stretch_', string(sf), '_take_', string(take_n), '.mat'));
%         outputFile = fullfile("data/networks/full-nets", strcat('pre-ablationtest_', string(take_n), '.mat'));
        save(outputFile, 'net', 'info');
   end
% 
   all_losses = [all_losses; subgroup_losses]
   all_RMSEs = [all_RMSEs; subgroup_RMSEs]

end

disp("Final Losses:")
disp(all_losses)
disp("Final RMSEs:")
disp(all_RMSEs)
% beep

% end



