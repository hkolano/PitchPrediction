load('data/full-data-matlab/FullData_NoVehXYZ_noB_noWaypoints_081522.mat')  

k = 40;
numChannels = size(XTrain{1}, 1);
pitch_idx = 14;
layers = setup_lookahead_rnn(numChannels, k, 1024);

for n = 1:numel(XTrain)
    resp = [];
    for t = 1:length(XTrain{n})-k
        resp = [resp, XTrain{n}(pitch_idx, t+1:t+k)'];
    end
    Resp_Train{n} = resp;
    XTrain{n} = XTrain{n}(:,1:end-k);
end

for n = 1:numel(XTest)
    resp = [];
    for t = 1:length(XTest{n})-k
        resp = [resp, XTest{n}(pitch_idx, t+1:t+k)'];
    end
    Resp_Test{n} = resp;
    XTest{n} = XTest{n}(:,1:end-k);
end

%     MaxEpochs=5, ...
% ValidationPatience = 2
init_options = trainingOptions("adam", ...
    MiniBatchSize=16, ...
    SequencePaddingDirection="right", ...
    Plots="training-progress", ...
    Shuffle='never', ...
    ValidationData={XTest, Resp_Test}, ...
    ValidationPatience = 5, ...
    ValidationFrequency = 50, ...
    OutputNetwork = 'best-validation-loss');
[net, info] = trainNetwork(XTrain,Resp_Train,layers,init_options);
%     
outputFile = fullfile("data/networks/full-nets/Z_3_nets", strcat('Z_3_', string(k), 'steps.mat'));
save(outputFile, 'net', 'info');