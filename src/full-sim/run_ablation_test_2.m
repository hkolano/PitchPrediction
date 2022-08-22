% load('data/full-data-matlab/FullData_NoVehXYZ_noB_noWaypoints_081522.mat') 
load('data/channel_dict.mat')
fields = {'pitch', 'dt', 'manip_vels'};
chan_idxs = rmfield(chan_idxs, fields);
fn = fieldnames(chan_idxs);

% Initialize constants
% ks =[5, 10, 15, 20, 25, 30, 40, 50, 60, 70, 75, 80, 90, 100, 125, 150, 175, 200];
k = 25;


for idx = 1:8
    load(strcat("data/full-data-matlab/channel_subgroups/no_manip_vels/data_without_", fn{idx}, ".mat"))
    numChannels = size(XTrain{1}, 1);
%     layers = layerGraph(net);
    clear Resp_Train Inputs_Train Resp_Test Inputs_Test

    layers = setup_lookahead_rnn(numChannels, k, 1024);
    
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
        MaxEpochs = 50, ...
        MiniBatchSize=16, ...
        SequencePaddingDirection="right", ...
        Plots="training-progress", ...
        Shuffle='never', ...
        ValidationData={Inputs_Test, Resp_Test}, ...
        ValidationFrequency = 50, ...
        OutputNetwork='best-validation-loss');
    [net, info] = trainNetwork(Inputs_Train,Resp_Train,layers,init_options);
    %     
    outputFile = fullfile("data/networks/full-nets/abl_2_nets", strcat('abl_2_no_', fn{idx}, '.mat'));
    save(outputFile, 'net', 'info');
end

