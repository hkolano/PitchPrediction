load('data/full-data-matlab/FullData_NoVehXYZ_noB_noWaypoints_081222.mat')  

k = 150;
numChannels = size(XTrain{1}, 1);
pitch_idx = 14;
layers = setup_lookahead_rnn(numChannels, k, 1024);

for n = 1:numel(XTrain)
    resp = [];
%     disp(length(XTrain{n}))
    if length(XTrain{n}) > 1.1*k
        for t = 1:length(XTrain{n})-k
            resp = [resp, XTrain{n}(pitch_idx, t+1:t+k)'];
        end
        Resp_Train{n} = resp;
        Inputs_Train{n} = XTrain{n}(:,1:end-k);
    end
end

for n = 1:numel(XTest)
    resp = [];
    if length(XTest{n}) > 1.1*k
        for t = 1:length(XTest{n})-k
            resp = [resp, XTest{n}(pitch_idx, t+1:t+k)'];
        end
        Resp_Test{n} = resp;
        Inputs_Test{n} = XTest{n}(:,1:end-k);
    end
end

%     MaxEpochs=5, ...
% ValidationPatience = 2
init_options = trainingOptions("adam", ...
    MiniBatchSize=16, ...
    SequencePaddingDirection="right", ...
    Plots="training-progress", ...
    Shuffle='never', ...
    ValidationData={Inputs_Test, Resp_Test}, ...
    ValidationPatience = 5, ...
    ValidationFrequency = 50);
[net, info] = trainNetwork(Inputs_Train,Resp_Train,layers,init_options);
%     
outputFile = fullfile("data/networks/full-nets/Z_3_nets", strcat('Z_3_', string(k), 'steps.mat'));
save(outputFile, 'net', 'info');