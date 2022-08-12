load('data/full-data-matlab/FullData_NoVehXYZ_noB_noWaypoints_081222.mat')  
load('data/networks/full-nets/KstepPreds/OneStepNet_fromScratch_128units.mat')

lgraph = layerGraph(net);

k = 2;
numChannels = size(XTrain{1}, 1);
pitch_idx = 14;

replacement_fcn = fullyConnectedLayer(k, "Name", "fc2out");
lgraph2 = replaceLayer(lgraph, "fc", replacement_fcn);

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
init_options = trainingOptions("adam", ...
    InitialLearnRate=0.0005, ...
    MiniBatchSize=20, ...
    SequencePaddingDirection="right", ...
    Plots="training-progress", ...
    Shuffle='never', ...
    ValidationData={XTest, Resp_Test}, ...
    ValidationFrequency = 100, ...
    ValidationPatience = 2);
[net, info] = trainNetwork(XTrain,Resp_Train,lgraph2,init_options);
%     
outputFile = fullfile("data/networks/full-nets/KstepPreds", 'OneStepNet_fromScratch_128units.mat');
save(outputFile, 'net', 'info');