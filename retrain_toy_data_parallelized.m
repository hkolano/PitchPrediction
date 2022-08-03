%% Import data
load('data/networks/toy-nets/SingleStepNet_080122.mat')
load('data/toy-data-matlab/TestandTrainData_080122.mat')
k = 25;     % Number of time steps to forecast (0.5s)

retrain_options = trainingOptions("adam", ...
    InitialLearnRate=0.001,...
    MaxEpochs=1, ...
    MiniBatchSize=5, ...
    SequencePaddingDirection="right",...
    ExecutionEnvironment="auto");

%% Define a validation set
load('data/toy-data-matlab/retrain_validation_vals_071322_v2.mat');
% num_vals = 20;
% val_idxs = [];
% val_ns = [];
% for i = 1:num_vals
%     val_idxs(i) = randi(size(XTest,2));
%     val_ns(i) = randi([2, size(XTest{val_idxs(i)}, 2)-k-1]);
% end

% outputFile = fullfile("data/toy-data-matlab", 'retrain_validation_vals_071322_v2.mat');
% save(outputFile, 'val_idxs', 'val_ns');
XTest_subset = XTest(val_idxs);
validate = @(net) validate_net(net, XTest_subset, val_ns, k, p);


%% Train on predictions
error = validate(net)
error_vec = [error];
training_rmse_vec = [];
% pred = toy_forecast(net, XTest{1}, 100, 25, p, true);
%%
for retrain_idx = 1:400

    % Extract only the data needed to be sent to workers
    traj_indices = [];
    trajs = {};
    for it_num = 1:50
        traj_idx = randi(size(XTrain, 2));
        traj_indices = [traj_indices traj_idx];
        trajs{it_num} = XTrain{traj_idx};
    end
    
    % Make 20 predictions in parallel
    parfor it_num = 1:50
        data = trajs{it_num};
        top_n_lim = size(data, 2) - k - 1;
        if top_n_lim <= k + 1
            n = top_n_lim;
        else
            n = randi([k, top_n_lim]);
        end
        
        % Generate a prediction
        pred = toy_forecast(net, data, n, k, p, false);

        wp_array = repmat(data(7:end,1), 1, length(pred));
        preds{it_num} = [pred; wp_array];
        g_truth{it_num} = data(1:6,2:n+k);
    end
    
    % Train on those predictions
    [net,info] = trainNetwork(preds, g_truth, layerGraph(net), retrain_options);
    this_test_rmse = info.TrainingRMSE;
    training_rmse_vec = [training_rmse_vec this_test_rmse];
    
    % Validate every so often
    if rem(retrain_idx, 25) == 0
        error = validate(net)
        disp(retrain_idx)
        error_vec = [error_vec error];
    end
    
end

% pred = toy_forecast(net, XTest{1}, 100, 25, p, true);

% Save the output
outputFile = fullfile("data/networks/toy-nets", 'retrained_p1_2_20k.mat');
save(outputFile, 'net', 'error_vec', 'training_rmse_vec');
%%
function error = validate_net(net, X_test, ns, k, p)
    error = 0;
    rmses = {};
    parfor i = 1:numel(X_test)
        pred = toy_forecast(net, X_test{i}, ns(i), k, p, false);
%         size(pred)
        pred = pred(:,end-k+1:end);
        g_truth = X_test{i}(1:6,ns(i)+1:ns(i)+k);
        rmse = sqrt(immse(pred, single(g_truth)));
        error = error + rmse;
    end
    error = error/numel(X_test);
end