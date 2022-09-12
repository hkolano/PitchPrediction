%% Import data

% Load data set
% load('data/full-data-matlab/channel_subgroups/no_goal_poses/no_manip_vels/no_goal_vels/data_without_xyz_poses.mat')
% 
% % Load network to retrain
% load('data/networks/full-nets/SingleStepNet_18chan_384units.mat')
% 
% % Load validation set definition
% load('data/full-data-matlab/val_set_post_abl.mat')


%% Initialization

k = 25;     % Number of time steps to forecast (0.5s)
mbatch = 4;
num_trajs_before_update = 16;
val_freq = 25;
save_freq = 250;
train_to_val_ratio = val_freq*(num_trajs_before_update/mbatch);

retrain_options = trainingOptions("adam", ...
    InitialLearnRate=0.001,...
    MaxEpochs=1, ...
    MiniBatchSize=mbatch, ...
    SequencePaddingDirection="right",...
    GradientThresholdMethod = 'l2norm',...
    GradientThreshold=1.0,...
    ExecutionEnvironment="auto");

num_rec_vars = size(TTest{1}, 1);

XTest_subset = XTest(val_idxs);
validate = @(net) validate_net(net, XTest_subset, val_ns, k, p, num_rec_vars);
validate_pitch = @(net) validate_pitch_only(net, XTest_subset, val_ns, k, p, num_rec_vars);

%%
% load('data/networks/full-nets/A_1_nets/net_A_1_58000its.mat')
% disp("New Network:")
% % mean(training_rmse_vec(end-100:end))
% % mean(error_vec(end-4:end))
% validate_pitch(net)


%% Train on predictions
first_error = validate(net)
error_vec = [];
training_rmse_vec = [];

for retrain_idx = 1:296

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

        wp_array = repmat(data(num_rec_vars+1:end,1), 1, size(pred, 2));
        preds{it_num} = [pred; wp_array];
        g_truth{it_num} = data(1:num_rec_vars,2:n+k);
    end

    nan_idxs = ID_nans(preds);
    if sum(nan_idxs) ~= 0
        disp(nan_idxs)
        disp("NaNs detected in prediction!")
    end
    
    [net,info] = trainNetwork(preds, g_truth, layerGraph(net), retrain_options);
    this_test_rmse = info.TrainingRMSE;
    training_rmse_vec = [training_rmse_vec this_test_rmse];
    
    if rem(retrain_idx, val_freq) == 0
        error = validate(net)
        disp(retrain_idx)
        error_vec = [error_vec error];
%         plot_errors(training_rmse_vec, error_vec, train_to_val_ratio, "Training Progress")
    end

%     if rem(retrain_idx, save_freq) == 0
%         outputFile = fullfile("data/networks/full-nets/A_3_nets", strcat("net_A_3_", string(retrain_idx), "its.mat"));
%         end_it = retrain_idx;
%         start_it = end_it-save_freq+1;
%         save(outputFile, 'net', 'info', "error_vec", "training_rmse_vec", "train_to_val_ratio", "start_it", "end_it")
%         plot_network_error_progression('data/networks/full-nets/A_3_nets', "A-3 Training Progression")
%         error_vec = [];
%         training_rmse_vec = [];
%     end
    
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

function error = validate_pitch_only(net, X_test, ns, k, p, num_recur)
    error = 0;
    for i = 1:numel(X_test)
        pred = full_forecast(net, X_test{i}, ns(i), k, p, num_recur, false);
%         size(pred)
        pred = pred(14,end-k+1:end);
        g_truth = X_test{i}(14,ns(i)+1:ns(i)+k);
        rmse = sqrt(immse(pred, single(g_truth)));
        error = error + rmse;
    end
    error = error/numel(X_test);
end