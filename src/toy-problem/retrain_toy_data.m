% Choose a random trajectory index to predict on
load('data/networks/toy-nets/SingleStepNet_071222_v1.mat')
load('data/toy-data-matlab/TestandTrainData_071222.mat')
n = 25;    % Number of time steps before starting forecasting
k = 25;     % Number of time steps to forecast (0.5s)

retrain_options = trainingOptions("adam", ...
    MaxEpochs=1, ...
    MiniBatchSize=20, ...
    SequencePaddingDirection="right");

for retrain_idx = 1:1

%         pred = toy_forecast(new_net, XTest{1}, n, k, true);

    for it_num = 1:20
        traj_idx = randi(size(XTrain, 2));
        n = randi(size(XTrain{traj_idx}, 2) - k - 1);
        % Generate a prediction
        pred = toy_forecast(new_net, XTrain{traj_idx}, n, k, false);

        wp_array = repmat(wp_dataTrain(:,traj_idx), 1, length(pred));
        preds{it_num} = [pred; wp_array];
        g_truth{it_num} = XTrain{traj_idx}(1:6,n+1:n+k+1);        
    end 

    new_net = trainNetwork(preds, g_truth, layerGraph(new_net), retrain_options);

end

n = randi(size(XTest{1}, 2) - k - 1);
pred = toy_forecast(new_net, XTest{1}, n, k, true);


%% Save the output
%     outputFile = fullfile("data/networks/toy-nets", 'retrained_net_071222.mat');
%     save(outputFile, 'new_net');