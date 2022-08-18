load('data/full-data-matlab/FullData_NoVehXYZ_noB_noWaypoints_081522.mat')  

% Sort validation data by sequence length
for i = 1:numel(XTest)
    sequenceLengths(i) = size(XTest{i}, 2);
end
[sequenceLengths, idx] = sort(sequenceLengths, 'descend');
XTest = XTest(idx);

% Initialize constants
ks = [200]; %[5, 10, 15]; %, 20, 25, 30, 40, 50, 60, 70, 75, 80, 90, 100, 125, 150, 175, 200];
numChannels = size(XTrain{1}, 1);
pitch_idx = 14;

for k_idx = 1:length(ks)
    k = ks(k_idx);
    load(strcat("data/networks/full-nets/Z_6_nets/Z_6_", string(k), "steps.mat"))
    layers = layerGraph(net);
    clear Resp_Train Inputs_Train Resp_Test Inputs_Test

%     layers = setup_lookahead_rnn(numChannels, k, 1024);
    
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
        MaxEpochs = 15, ...
        MiniBatchSize=16, ...
        SequencePaddingDirection="right", ...
        Plots="training-progress", ...
        Shuffle='never', ...
        ValidationData={Inputs_Test, Resp_Test}, ...
        ValidationFrequency = 50, ...
        OutputNetwork='best-validation-loss');
    [net, info] = trainNetwork(Inputs_Train,Resp_Train,layers,init_options);
    %     
    outputFile = fullfile("data/networks/full-nets/Z_7_nets", strcat('Z_7_', string(k), 'steps.mat'));
    save(outputFile, 'net', 'info');
end

