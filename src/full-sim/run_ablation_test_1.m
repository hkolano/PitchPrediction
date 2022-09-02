% load('data/full-data-matlab/FullData_NoVehXYZ_noB_noWaypoints_081522.mat') 
load('data/channel_dict.mat')
chan_idxs = rmfield(chan_idxs, 'pitch');
chan_idxs = rmfield(chan_idxs, 'dt');

elimd_gps = ["goal_poses", "manip_vels", "goal_vels", "xyz_poses", "manip_des_vels"];
path = "data/full-data-matlab/channel_subgroups";
level_num = length(elimd_gps)+1;

for rnd_num = 1:length(elimd_gps)
    rnd_name = elimd_gps(rnd_num);
    chan_idxs = rmfield(chan_idxs, rnd_name);
    path = strcat(path, "/no_", elimd_gps(rnd_num));
end
fn = fieldnames(chan_idxs);

% Initialize constants
% ks =[5, 10, 15, 20, 25, 30, 40, 50, 60, 70, 75, 80, 90, 100, 125, 150, 175, 200];
k = 25;
numUnits = 128;

all_losses = [];
subgroup_losses = [];
all_RMSEs = [];
subgroup_RMSEs = [];

for idx = 1:length(fn)
%     load("data/full-data-matlab/FullData_081022.mat")
    load(strcat(path, "/data_without_", fn{idx}, ".mat"))
    numChannels = size(XTrain{1}, 1);
%     layers = layerGraph(net);
    clear Resp_Train Inputs_Train Resp_Test Inputs_Test

    layers = setup_lookahead_rnn(numChannels, k, numUnits);
    subgroup_losses = [];
    subgroup_RMSEs = [];

    for n = 1:numel(XTrain)
        resp = zeros(k, size(XTrain{n}, 2)-k);
        if size(XTrain{n}, 2) > 1.1*k
            for t = 1:size(XTrain{n}, 2)-k
                resp(:,t) = XTrain{n}(pitch_idx, t+1:t+k)';
            end
            Resp_Train{n} = resp;
            Inputs_Train{n} = XTrain{n}(:,1:end-k);
        end
    end
    
    for n = 1:numel(XTest)
        resp = zeros(k, size(XTest{n}, 2)-k);
        if size(XTest{n}, 2) > 1.1*k
            for t = 1:size(XTest{n}, 2)-k
                resp(:,t) = XTest{n}(pitch_idx, t+1:t+k)';
            end
            Resp_Test{n} = resp;
            Inputs_Test{n} = XTest{n}(:,1:end-k);
        end
    end
    
    init_options = trainingOptions("adam", ...
        InitialLearnRate=0.001,...
        LearnRateDropPeriod=5, ...
        LearnRateSchedule='piecewise', ...
        LearnRateDropFactor=.8, ...
        MaxEpochs = 50, ...
        MiniBatchSize=16, ...
        SequencePaddingDirection="right", ...
        Plots="training-progress", ...
        Shuffle='every-epoch', ...
        ValidationData={Inputs_Test, Resp_Test}, ...
        ValidationFrequency = 60, ...
        OutputNetwork='best-validation-loss');

   for take_n = 1:3
        [net, info] = trainNetwork(Inputs_Train,Resp_Train,layers,init_options);
        
        subgroup_losses = [subgroup_losses, info.FinalValidationLoss];
        subgroup_RMSEs = [subgroup_RMSEs, info.FinalValidationRMSE];
        %     
        outputFile = fullfile(strcat("data/networks/full-nets/ablTake2_", string(level_num), "_nets"), strcat('abl_', string(level_num), '_no_', fn{idx}, '_take', string(take_n), '_droprate.mat'));
%         outputFile = fullfile("data/networks/full-nets", strcat('pre-ablationtest_', string(take_n), '.mat'));
        save(outputFile, 'net', 'info');
   end

   all_losses = [all_losses; subgroup_losses];
   all_RMSEs = [all_RMSEs; subgroup_RMSEs];
end

disp("Final Losses:")
disp(all_losses)
disp("Final RMSEs:")
disp(all_RMSEs)
beep

% end



