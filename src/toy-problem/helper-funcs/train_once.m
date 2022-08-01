load('data/toy-data-matlab/TestandTrainData_WithDesVels_080122.mat')  

numChannels = size(XTrain{1}, 1);
[layers, options] = setup_residual_gru(numChannels, XTest, TTest);

init_options = trainingOptions("adam", ...
    MiniBatchSize=20, ...
    SequencePaddingDirection="right", ...
    Plots="training-progress", ...
    Shuffle='never', ...
    ValidationData={XTest, TTest}, ...
    ValidationFrequency = 250, ...
    ValidationPatience = 2);
net = trainNetwork(XTrain,TTrain,layers,init_options);
    
outputFile = fullfile("data/networks/toy-nets/with-desvels", 'SingleStepNet_toConvergence_080122.mat');
save(outputFile, 'net');