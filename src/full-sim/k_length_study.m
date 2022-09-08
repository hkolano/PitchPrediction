%% Setup
% load('data/full-data-matlab/FullData_NoVehXYZ_noB_noWaypoints_081522.mat') 
load('data/channel_dict.mat')
chan_idxs = rmfield(chan_idxs, 'pitch');
chan_idxs = rmfield(chan_idxs, 'dt');

elimd_gps = ["goal_poses", "manip_vels", "goal_vels"];
path = "data/full-data-matlab/channel_subgroups";
level_num = length(elimd_gps)+1;

for rnd_num = 1:length(elimd_gps)
    rnd_name = elimd_gps(rnd_num);
    chan_idxs = rmfield(chan_idxs, rnd_name);
    path = strcat(path, "/no_", elimd_gps(rnd_num));
end
fn = fieldnames(chan_idxs);

load(fullfile(path, "data_without_xyz_poses.mat"))
numChannels = size(XTrain{1}, 1);

% Initialize constants
% ks =[5, 10, 15, 20, 25, 30, 40, 50, 60, 70, 75, 80, 90, 100, 125, 150, 175, 200];
k = 25;
numUnits = 384;
% stretches = [1, 2, 3, 4, 5, 6];
stretches = [7, 8, 9];

all_losses = [];
subgroup_losses = [];
all_RMSEs = [];
subgroup_RMSEs = [];

for idx = 1:length(stretches)
%% Input/Responses setup
    clear Resp_Train Inputs_Train Resp_Test Inputs_Test

    sf = stretches(idx);

    layers = setup_lookahead_rnn(numChannels, k, numUnits);
    subgroup_losses = [];
    subgroup_RMSEs = [];

    for n = 1:numel(XTrain)
        resp = zeros(k, size(XTrain{n}, 2)-sf*k);
        if size(XTrain{n}, 2) > (sf+.1)*k
            for t = 1:size(XTrain{n}, 2)-sf*k
                resp(:,t) = XTrain{n}(pitch_idx, t+sf:sf:t+sf*k)';
            end
            Resp_Train{n} = resp;
            Inputs_Train{n} = XTrain{n}(:,1:end-sf*k);
        end
    end
    
    for n = 1:numel(XTest)
        resp = zeros(k, size(XTest{n}, 2)-sf*k);
        if size(XTest{n}, 2) > (sf+.1)*k
            for t = 1:size(XTest{n}, 2)-sf*k
                resp(:,t) = XTest{n}(pitch_idx, t+sf:sf:t+sf*k)';
            end
            Resp_Test{n} = resp;
            Inputs_Test{n} = XTest{n}(:,1:end-sf*k);
        end
    end
    
    init_options = trainingOptions("adam", ...
        InitialLearnRate=0.001,...
        LearnRateDropPeriod=5, ...
        LearnRateSchedule='piecewise', ...
        LearnRateDropFactor=.9, ...
        MaxEpochs = 100, ...
        MiniBatchSize=16, ...
        SequencePaddingDirection="right", ...
        Plots="training-progress", ...
        Shuffle='every-epoch', ...
        ValidationData={Inputs_Test, Resp_Test}, ...
        ValidationFrequency = 60, ...
        OutputNetwork='best-validation-loss');

    %% Train the net
   for take_n = 1:3
        [net, info] = trainNetwork(Inputs_Train,Resp_Train,layers,init_options);
        
        subgroup_losses = [subgroup_losses, info.FinalValidationLoss];
        subgroup_RMSEs = [subgroup_RMSEs, info.FinalValidationRMSE];
        %     
        outputFile = fullfile("data/networks/full-nets/simple_w_stretch_factor", strcat('stretch_', string(sf), '_take_', string(take_n), '.mat'));
%         outputFile = fullfile("data/networks/full-nets", strcat('pre-ablationtest_', string(take_n), '.mat'));
        save(outputFile, 'net', 'info');
   end

   all_losses = [all_losses; subgroup_losses]
   all_RMSEs = [all_RMSEs; subgroup_RMSEs]

end

disp("Final Losses:")
disp(all_losses)
disp("Final RMSEs:")
disp(all_RMSEs)
beep

% end



