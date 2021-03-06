load('data/toy-data-matlab/TestandTrainData_080122.mat')  

numChannels = size(XTrain{1}, 1);
[layers, options] = setup_residual_gru(numChannels, XTest, TTest);

init_options = trainingOptions("adam", ...
    MaxEpochs=1, ...
    MiniBatchSize=20, ...
    SequencePaddingDirection="right", ...
    Plots="training-progress", ...
    Shuffle='never', ...
    ValidationData={XTest, TTest}, ...
    ValidationFrequency = 250, ...
    ValidationPatience = 2);
net = trainNetwork(XTrain,TTrain,layers,init_options);
    
outputFile = fullfile("data/networks/toy-nets", 'SingleStepNet_080222.mat');
save(outputFile, 'net');