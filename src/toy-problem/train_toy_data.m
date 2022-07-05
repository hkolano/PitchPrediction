function train_toy_data()
    data = import_toy_training_data("data/toy-data");


    % Divide into train and test data
    numObservations = numel(data);
    idxTrain = 1:floor(0.9*numObservations);
    idxTest = floor(0.9*numObservations)+1:numObservations;
    dataTrain = data(idxTrain);
    dataTest = data(idxTest);

    numChannels = size(dataTrain{1}, 1);

    for n = 1:numel(dataTrain)
        X = dataTrain{n};
        XTrain{n} = X(:,1:end-1);
        TTrain{n} = X(2:end,2:end);
    end

    for n = 1:numel(dataTest)
        X = dataTest{n};
        XTest{n} = X(:,1:end-1);
        TTest{n} = X(2:end,2:end);
    end

    layers = [
        sequenceInputLayer(numChannels)
        lstmLayer(128, 'OutputMode', 'sequence')
        fullyConnectedLayer(6)
        regressionLayer];

    options = trainingOptions("adam", ...
        MaxEpochs=100, ...
        SequencePaddingDirection="left", ...
        Shuffle="every-epoch", ...
        Plots="training-progress", ...
        Verbose=0, ...
        ValidationData={XTest, TTest}, ...
        ValidationFrequency = 25);
    
%     disp(XTrain{1})

%     net = trainNetwork(XTrain,TTrain,layers,options);


%     outputFile = fullfile("data/networks/toy-nets", 'netv1.mat');
%     save(outputFile, 'net');
%     
%     outputFile2 = fullfile("data/networks/toy-nets", 'netv1testdata.mat');
%     save(outputFile2, 'XTest', 'TTest')

end

% Net v1:
%     layers = [
%         sequenceInputLayer(numChannels)
%         lstmLayer(128)
%         fullyConnectedLayer(1)
%         regressionLayer];

% Net v2:
%     layers = [
%         sequenceInputLayer(numChannels)
%         lstmLayer(128, 'OutputMode', 'sequence')
%         fullyConnectedLayer(1)
%         regressionLayer];