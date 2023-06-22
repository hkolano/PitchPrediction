%{
Generates the pitch-only prediction networks. Requires that the data be 
normalized and chan_idxs be defined in a file. Does 3 runs per level. 

Make sure data input is the correct one, and that MaxEpochs is set to 100 in
init_opts. Make sure it is saving results to a good folder. 

Last modified 3/8/23
%}
%% Setup
% Load data set
load("data/full-sim-data-110822/FullData.mat")

% get indices of remaining feature groups
elimd_gps = ["xyz_poses", "xyz_vels", "goal_poses", "manip_des_vels", "goal_vels"];
all_idxs = get_remaining_idxs(elimd_gps);

%%
% Initialize constants
% ks =[5, 10, 15, 20, 25, 30, 40, 50, 60, 70, 75, 80, 90, 100, 125, 150, 175, 200];
k = 25;
pitch_idx = 23;
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
        MaxEpochs = 100, ...
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
        outputFile = fullfile("data/networks/icra-redo-nets/simple_w_stretch_factor", strcat('stretch_', string(sf), '_take_', string(take_n), '.mat'));
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



