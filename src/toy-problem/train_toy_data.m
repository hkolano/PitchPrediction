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
    
    %{
    X(:,1) is a column vector. 
    1-3: joint values (pitch, J1, J2) from sequence 
    4-6: joint velocities (pitch, J1, J2) from sequence
    7: time step of data collection (constant)
    8-9: known starting positions (constant)
    10-11: goal end positions (constant)
    12-13: known starting velocities (constant)
    14-15: goal end velocities (constant)
    X(:,1:6) are fed to the LSTM, while X(:,7:end) are sent directly to the
    FCN.
    %}

    %% Initial network setup
    [layers, options] = setup_rnn(numChannels, XTest, TTest);

    init_options = trainingOptions("adam", ...
        MaxEpochs=2, ...
        MiniBatchSize=1, ...
        SequencePaddingDirection="right");
    net = trainNetwork(XTrain,TTrain,layers,init_options);
        
    %% Retraining
    % Choose a random trajectory index to predict on

        n = 100;    % Number of time steps before starting forecasting
        k = 25;     % Number of time steps to forecast (0.5s)

    for traj = 1:20
        % Generate a prediction
        pred = toy_forecast(net, XTrain{traj_idx}, n, k);

        wp_array = repmat(wp_dataTrain(:,traj_idx), 1, length(pred));
        preds{1} = [pred; wp_array];
        g_truth = XTrain{traj_idx}(1:6,n+1:n+k+1);

        new_lgraph = layerGraph(net);
        net = trainNetwork(preds, {g_truth}, new_lgraph, init_options);
    end 
    %% Save the output
    outputFile = fullfile("data/networks/toy-nets", 'netv2_1.mat');
    save(outputFile, 'net');
    
    outputFile2 = fullfile("data/networks/toy-nets", 'netv2_1testdata.mat');
    save(outputFile2, 'XTest', 'TTest')

% end

