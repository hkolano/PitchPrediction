%% Import data

% Load data set
load('data/full-data-matlab/FullData_NoVehXYZ_noB_081022.mat')

% Load network to retrain
load('data/networks/full-nets/SingleStepNet_fromFullData_NoVehXYZ_noB_1028units.mat')

% Load validation set definition
load('data/full-data-matlab/val_set_081122.mat')


%% Initialization

k = 25;     % Number of time steps to forecast (0.5s)

retrain_options = trainingOptions("adam", ...
    InitialLearnRate=0.001,...
    MaxEpochs=1, ...
    MiniBatchSize=4, ...
    SequencePaddingDirection="right",...
    ExecutionEnvironment="auto");

validate = @(net) validate_net(net, XTest, val_idxs, val_ns, k, p);


%% Train on predictions
error = validate(net)
error_vec = [error];
training_rmse_vec = [];
% pred = toy_forecast(net, XTest{1}, 100, 25, p, true);

for retrain_idx = 1:10
    
    for it_num = 1:5
        traj_idx = randi(size(XTrain, 2));
        top_n_lim = size(XTrain{traj_idx}, 2) - k - 1;
        if top_n_lim <= k + 1
            n = top_n_lim;
        else
            n = randi([k, top_n_lim]);
        end
        
        % Generate a prediction
        pred = toy_forecast(net, XTrain{traj_idx}, n, k, p, false);

        wp_array = repmat(XTrain{traj_idx}(7:end,1), 1, length(pred));
        preds{it_num} = [pred; wp_array];
        g_truth{it_num} = XTrain{traj_idx}(1:6,2:n+k);
    end
    
    [net,info] = trainNetwork(preds, g_truth, layerGraph(net), retrain_options);
    this_test_rmse = info.TrainingRMSE;
    training_rmse_vec = [training_rmse_vec this_test_rmse];
    
    if rem(retrain_idx, 10) == 0
        error = validate(net)
        disp(retrain_idx)
        error_vec = [error_vec error];
    end
    
end

% pred = toy_forecast(net, XTest{1}, 100, 25, p, true);

% Save the output
outputFile = fullfile("data/networks/toy-nets", 'retrained_072022_v6_2.mat');
save(outputFile, 'net', 'error_vec', 'training_rmse_vec');

function error = validate_net(net, X_test, idxs, ns, k, p)
    error = 0;
    for i = 1:length(idxs)
        pred = toy_forecast(net, X_test{idxs(i)}, ns(i), k, p, false);
%         size(pred)
        pred = pred(:,end-k+1:end);
        g_truth = X_test{idxs(i)}(1:6,ns(i)+1:ns(i)+k);
        rmse = sqrt(immse(pred, single(g_truth)));
        error = error + rmse;
    end
    error = error/length(idxs);
end