%% Setup
load("data/full-sim-data-022223/FullData_10Hz.mat")
% XTrain = XTrain_10hz;
% XTest = XTest_10hz;

%%
load("data/full-sim-data-022223/channel_dict.mat")
pitch_idx = chan_idxs.act_pitch;
elimd_gps = ["meas_rpy"];
all_idxs = get_remaining_idxs(elimd_gps, chan_idxs);

%%
% Initialize constants
% ks =[5, 10, 15, 20, 25, 30, 40, 50, 60, 70, 75, 80, 90, 100, 125, 150, 175, 200];
k = 1; %25
numUnits = 384;
stretches = [1];%, 2, 3, 4, 5, 6, 7, 8];
% stretches = [7, 8, 9];

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

    for n = 1:numel(XTrain)
        resp = zeros(k, size(XTrain{n}, 2)-sf*k);
        if size(XTrain{n}, 2) > (sf+.1)*k
            for t = 1:size(XTrain{n}, 2)-sf*k
                resp(:,t) = XTrain{n}(pitch_idx, t+sf:sf:t+sf*k)';
            end
%             Resp_Train{n} = resp;
            Inputs_Train{n} = XTrain{n}(all_idxs,1:end-sf*k-1);
            % ONLY FOR SINGLE STEP NET
            Resp_Train{n} = XTrain{n}(all_idxs, 2:end-sf*k);
        end
%         disp(size(XTrain{n}, 2))
    end
    
    for n = 1:numel(XTest)
        resp = zeros(k, size(XTest{n}, 2)-sf*k);
        if size(XTest{n}, 2) > (sf+.1)*k
            for t = 1:size(XTest{n}, 2)-sf*k
                resp(:,t) = XTest{n}(pitch_idx, t+sf:sf:t+sf*k)';
            end
%             Resp_Test{n} = resp;
            Inputs_Test{n} = XTest{n}(all_idxs,1:end-sf*k-1);
            % ONLY FOR SINGLE STEP NET
            Resp_Test{n} = XTest{n}(all_idxs, 2:end-sf*k);
        end
    end

    numChannels = size(Inputs_Train{1}, 1);
    layers = setup_lookahead_rnn(numChannels, numChannels, numUnits);
    
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
        ValidationData={Inputs_Test, Resp_Test}, ...
        ValidationFrequency = 60, ...
        OutputNetwork='best-validation-loss');

    %% Train the net
   for take_n = 1:1
        [net, info] = trainNetwork(Inputs_Train,Resp_Train,layers,init_options);
        
        subgroup_losses = [subgroup_losses, info.FinalValidationLoss];
        subgroup_RMSEs = [subgroup_RMSEs, info.FinalValidationRMSE];
        %     
        outputFile = fullfile("data/networks/iros-nets", "SingleStepNet_10Hz.mat")
%         outputFile = fullfile("data/networks/icra-redo-nets/simple_w_stretch_factor", strcat('stretch_', string(sf), '_take_', string(take_n), '.mat'));
%         outputFile = fullfile("data/networks/full-nets", strcat('pre-ablationtest_', string(take_n), '.mat'));
        save(outputFile, 'net', 'info');
   end
% 
%    all_losses = [all_losses; subgroup_losses]
%    all_RMSEs = [all_RMSEs; subgroup_RMSEs]

end

disp("Final Losses:")
disp(all_losses)
disp("Final RMSEs:")
disp(all_RMSEs)
% beep

% end



