% function train_toy_data()
%% Data import and processing
    sequence_data = import_toy_training_data("data/toy-data");
    waypoint_data = import_toy_waypoint_data();

    % Determine division between train and test data
    numObservations = numel(sequence_data);
    idxTrain = 1:floor(0.9*numObservations);
    idxTest = floor(0.9*numObservations)+1:numObservations;
    
    % Split sequence data
    seq_dataTrain = sequence_data(idxTrain);
    seq_dataTest = sequence_data(idxTest);
    wp_dataTrain = waypoint_data(:,idxTrain);
    wp_dataTest = waypoint_data(:,idxTest);
    
    numChannels = size(seq_dataTrain{1}, 1);

    for n = 1:numel(seq_dataTrain)
        X = seq_dataTrain{n};
        wp_array = repmat(wp_dataTrain(:,n), 1, length(X)-1);
        XTrain{n} = [X(:,1:end-1); wp_array];
        TTrain{n} = X(:,2:end);
    end

    for n = 1:numel(seq_dataTest)
        X = seq_dataTest{n};
        wp_array = repmat(wp_dataTest(:,n), 1, length(X)-1);
        XTest{n} = [X(:,1:end-1); wp_array];
        TTest{n} = X(:,2:end);
    end

    %% Network setup
    [layers, options] = setup_rnn(numChannels, XTest, TTest);

    init_options = trainingOptions("adam", ...
        MaxEpochs=1, ...
        MiniBatchSize=1, ...
        SequencePaddingDirection="right");
    net = trainNetwork(XTrain(1:5),TTrain(1:5),layers,init_options);
    
    new_lgraph = layerGraph(net);
    net = trainNetwork(XTrain(1:5), TTrain(1:5), new_lgraph, init_options);


    outputFile = fullfile("data/networks/toy-nets", 'netv2_1.mat');
    save(outputFile, 'net');
    
    outputFile2 = fullfile("data/networks/toy-nets", 'netv2_1testdata.mat');
    save(outputFile2, 'XTest', 'TTest')

% end

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