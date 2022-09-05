load('data/full-data-matlab/FullData_081022.mat') 
% load('data/channel_dict.mat')
% chan_idxs = rmfield(chan_idxs, 'pitch');
% chan_idxs = rmfield(chan_idxs, 'dt');
% fn = fieldnames(chan_idxs);

% Initialize constants
% ks =[5, 10, 15, 20, 25, 30, 40, 50, 60, 70, 75, 80, 90, 100, 125, 150, 175, 200];
k = 25;
numUnits_vec = [8, 32, 128, 256, 512, 1024];
pitch_idx = 14;
losses = [];
RMSEs = [];

% for idx = 1:length(numUnits_vec)
    numUnits = 128;
    disp(strcat("Training with ", string(numUnits), " GRU units"))
%     load(strcat("data/full-data-matlab/channel_subgroups/data_without_", fn{idx}, ".mat"))
    numChannels = size(XTrain{1}, 1);
%     layers = layerGraph(net);
    clear Resp_Train Inputs_Train Resp_Test Inputs_Test

    layers = setup_lookahead_rnn(numChannels, k, numUnits);
    
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
        
    %     MaxEpochs = 50, ...
    %         Shuffle='never', ...
    %         SequenceLength = "shortest", ...
    %         Shuffle = 'every-epoch', ...
    %         ValidationPatience = 25, ...
        init_options = trainingOptions("adam", ...
            InitialLearnRate=.002,...
            MiniBatchSize=16, ...
            MaxEpochs = 35, ...
            SequencePaddingDirection="right", ...
            Plots="training-progress", ...
            Shuffle = 'every-epoch', ...
            ValidationData={Inputs_Test, Resp_Test}, ...
            ValidationFrequency = 60, ...
            ValidationPatience = 25, ...
            OutputNetwork='best-validation-loss');

   for take_n = 1:2
        [net, info] = trainNetwork(Inputs_Train,Resp_Train,layers,init_options);
        
        losses = [losses, info.FinalValidationLoss];
        RMSEs = [RMSEs, info.FinalValidationRMSE];
        %     
        outputFile = fullfile("data/networks/full-nets/increasing_units_testing", strcat('k25_40epochs_', string(numUnits), 'units_take', string(take_n), '.mat'));
        save(outputFile, 'net', 'info');
    end
% end

disp("Final Losses:")
disp(losses)
disp("Final RMSEs:")
disp(RMSEs)