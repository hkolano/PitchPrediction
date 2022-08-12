%% Import data

% Load data set
load('data/full-data-matlab/FullData_NoVehXYZ_noB_081022.mat')

% Load network to retrain
load('data/networks/full-nets/retrained_nets/net_A_1_10000its.mat')

% Load validation set definition
load('data/full-data-matlab/val_set_081122.mat')


%% Initialization

k = 25;     % Number of time steps to forecast (0.5s)
mbatch = 4;
num_trajs_before_update = 20;
val_freq = 25;
save_freq = 500;
train_to_val_ratio = val_freq*(num_trajs_before_update/mbatch);

retrain_options = trainingOptions("adam", ...
    InitialLearnRate=0.001,...
    MaxEpochs=1, ...
    MiniBatchSize=mbatch, ...
    SequencePaddingDirection="right",...
    ExecutionEnvironment="auto");

num_rec_vars = size(TTest{1}, 1);

XTest_subset = XTest(val_idxs);
validate = @(net) validate_net(net, XTest_subset, val_ns, k, p, num_rec_vars);


%% Train on predictions
% error = validate(net)
% error_vec = [error];
% training_rmse_vec = [];

for retrain_idx = 10001:30000

    traj_indices = [];
    trajs = {};

    for it_num = 1:num_trajs_before_update
        traj_idx = randi(size(XTrain, 2));
        traj_indices = [traj_indices traj_idx];
        trajs{it_num} = XTrain{traj_idx};
    end
    
    parfor it_num = 1:num_trajs_before_update
        data = trajs{it_num};
        top_n_lim = size(data, 2) - k - 1;
        if top_n_lim <= k + 1
            n = top_n_lim;
        else
            n = randi([k, top_n_lim]);
        end
        
        % Generate a prediction
        pred = full_forecast(net, data, n, k, p, num_rec_vars, false);

        wp_array = repmat(data(num_rec_vars+1:end,1), 1, length(pred));
        preds{it_num} = [pred; wp_array];
        g_truth{it_num} = data(1:num_rec_vars,2:n+k);
    end
    
    [net,info] = trainNetwork(preds, g_truth, layerGraph(net), retrain_options);
    this_test_rmse = info.TrainingRMSE;
    training_rmse_vec = [training_rmse_vec this_test_rmse];
    
    if rem(retrain_idx, val_freq) == 0
        error = validate(net)
        disp(retrain_idx)
        error_vec = [error_vec error];
        plot_errors(training_rmse_vec, error_vec, train_to_val_ratio, "Training Progress")
    end

    if rem(retrain_idx, save_freq) == 0
        outputFile = fullfile("data/networks/full-nets/retrained_nets", strcat("net_A_1_", string(retrain_idx), "its.mat"));
        save(outputFile, 'net', 'info', "error_vec", "training_rmse_vec", "train_to_val_ratio")
    end
    
end

% pred = toy_forecast(net, XTest{1}, 100, 25, p, true);

% Save the output
% outputFile = fullfile("data/networks/full-nets/retrained_nets", 'test_net.mat');
% save(outputFile, 'net', 'info', 'error_vec', 'training_rmse_vec');

function error = validate_net(net, X_test, ns, k, p, num_recur)
    error = 0;
    parfor i = 1:numel(X_test)
        pred = full_forecast(net, X_test{i}, ns(i), k, p, num_recur, false);
%         size(pred)
        pred = pred(:,end-k+1:end);
        g_truth = X_test{i}(1:num_recur,ns(i)+1:ns(i)+k);
        rmse = sqrt(immse(pred, single(g_truth)));
        error = error + rmse;
    end
    error = error/numel(X_test);
end