% function train_toy_data()
%% Data import and processing
    sequence_data = import_toy_training_data("data/toy-data");
    waypoint_data = import_toy_waypoint_data();
    
    [sequence_data, p] = normalize_data(sequence_data);
    bad_ids = ID_outliers(sequence_data);
    
    for idx = 1:length(bad_ids)
        sequence_data(bad_ids(idx)) = [];
        waypoint_data(:, bad_ids(idx)) = [];
    end

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
    
    %% Sort by sequence length
    for i=1:numel(XTrain)
        sequence = XTrain{i};
        sequenceLengths(i) = size(sequence,2);
    end

    [sequenceLengths,idx] = sort(sequenceLengths,'descend');
    XTrain = XTrain(idx);
    TTrain = TTrain(idx);

    %{
    X(:,1) is a column vector. 
    1-3: joint values (pitch, J1, J2) from sequence NORMALIZED
    4-6: joint velocities (pitch, J1, J2) from sequence NORMALIZED
    7: time step of data collection (constant)
    8-9: known starting positions (constant)
    10-11: goal end positions (constant)
    12-13: known starting velocities (constant)
    14-15: goal end velocities (constant)
    X(:,1:6) are fed to the LSTM, while X(:,7:end) are sent directly to the
    FCN.
    %}

    %% Initial network setup
    [layers, options] = setup_recurrent_gru(numChannels, XTest, TTest);

    init_options = trainingOptions("adam", ...
        MaxEpochs=1, ...
        MiniBatchSize=20, ...
        SequencePaddingDirection="right", ...
        Plots="training-progress", ...
        Shuffle='never', ...
        ValidationData={XTest, TTest}, ...
        ValidationFrequency = 250, ...
        ValidationPatience = 1);
    net = trainNetwork(XTrain,TTrain,layers,init_options);
    
    outputFile = fullfile("data/networks/toy-nets", 'SingleStepNet_OneEpoch_071322_v2.mat');
    save(outputFile, 'net');
        
%     %% Retraining
%     % Choose a random trajectory index to predict on
%     new_net = net;
%     n = 25;    % Number of time steps before starting forecasting
%     k = 25;     % Number of time steps to forecast (0.5s)
%     
%     retrain_options = trainingOptions("adam", ...
%         MaxEpochs=1, ...
%         MiniBatchSize=20, ...
%         SequencePaddingDirection="right", ...
%         Shuffle='never');
%     
%     for retrain_idx = 1:1
%     
% %         pred = toy_forecast(new_net, XTest{1}, n, k, true);
% 
%         for it_num = 1:20
%             traj_idx = randi(size(XTrain, 2));
%             n = randi(size(XTrain{traj_idx}, 2) - k - 1);
%             % Generate a prediction
%             pred = toy_forecast(new_net, XTrain{traj_idx}, n, k, false);
% 
%             wp_array = repmat(wp_dataTrain(:,traj_idx), 1, length(pred));
%             preds{it_num} = [pred; wp_array];
%             g_truth{it_num} = XTrain{traj_idx}(1:6,n+1:n+k+1);        
%         end 
% 
%         new_net = trainNetwork(preds, g_truth, layerGraph(new_net), retrain_options);
%         
%     end
%     
%     n = randi(size(XTest{1}, 2) - k - 1);
%     pred = toy_forecast(new_net, XTest{1}, n, k, true);
%     

    %% Save the output
%     outputFile = fullfile("data/networks/toy-nets", 'netv2_2.mat');
%     save(outputFile, 'new_net');
%     
    outputFile2 = fullfile("data/toy-data-matlab", 'TestandTrainData_071322.mat');
    save(outputFile2, 'XTest', 'TTest', 'XTrain', 'TTrain', 'p')

% end

