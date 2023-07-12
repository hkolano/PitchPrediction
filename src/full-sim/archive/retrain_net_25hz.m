%% Import data

% Load data set
load('data/full-data-matlab/FullData_17chan_25Hz.mat')

% Load network to retrain
load('data/networks/full-nets/SingleStepNet_25hz_17chan_384units_alltrajs.mat')

% Load validation set definition
load('data/full-data-matlab/val_set_post_abl.mat')


%% Initialization

% k = 5;     % Number of time steps to forecast (0.5s)
ks = [12 25 50 75 100];
mbatch = 16;
num_trajs_before_update = 16;
val_freq = 50;
train_to_val_ratio = val_freq*(num_trajs_before_update/mbatch);
num_epochs = 2;
save_freq = 1; %epochs
init_learning_rate = .0005;

retrain_options = trainingOptions("adam", ...
    InitialLearnRate= init_learning_rate,...
    MaxEpochs=1, ...
    MiniBatchSize=mbatch, ...
    SequencePaddingDirection="right",...
    GradientThresholdMethod = 'l2norm',...
    GradientThreshold=0.9,...
    ExecutionEnvironment="auto");

XTest = XTest_25hz;
XTrain = XTrain_25hz;
TTest = TTest_25hz;
TTrain = TTrain_25hz;

num_rec_vars = size(TTest{1}, 1);

XTest_subset = XTest(val_idxs);
val_ns = floor(val_ns./2);
% validate = @(net) validate_net(net, XTest_subset, val_ns, k, p);
% validate_pitch = @(net) validate_pitch_only(net, XTest_subset, val_ns, k, p, num_rec_vars);


%% Train on predictions
for k_val = 1:length(ks)
    load('data/networks/full-nets/SingleStepNet_25hz_17chan_384units_alltrajs.mat')
    validate = @(net) validate_net(net, XTest_subset, val_ns, k, p);
    k = ks(k_val);
    first_error = validate(net)
    error_vec = [first_error];
    error_vec_its = [1];
    training_rmse_vec = []; %these_epochs_training_RMSE_vec; %[];
    total_it = 1;
    start_it = total_it;
    retrain_options.InitialLearnRate = init_learning_rate;

    tic
    
    for epoch_n = 1:num_epochs
    
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
                pred = full_forecast_norecur(net, data, n, k, p);
        
                preds{it_num} = pred;
                g_truth{it_num} = data(:,2:n+k);
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
                disp(total_it)
                toc
                error_vec = [error_vec error];
                error_vec_its = [error_vec_its total_it];
                plot_training(error_vec, error_vec_its, training_rmse_vec);
        %         plot_errors(training_rmse_vec, error_vec, train_to_val_ratio, "Training Progress")
            end
            total_it = total_it + 1;
        end
    
        if rem(epoch_n, save_freq) == 0 || epoch_n > 45
            outputFile = fullfile(strcat("data/networks/full-nets/25Hz_alltrajs_k", string(k)), strcat("take3_", string(epoch_n), "epochs.mat"));
            end_it = total_it-1;
            these_epochs_training_RMSE_vec = training_rmse_vec(start_it:end_it);
            save(outputFile, 'net', 'info', "error_vec", "error_vec_its", "these_epochs_training_RMSE_vec", "start_it", "end_it");
    %         plot_network_error_progression('data/networks/full-nets/A_3_nets', "A-3 Training Progression")
            start_it = total_it;
            retrain_options.InitialLearnRate = retrain_options.InitialLearnRate*.9;
        end
    
    end
end

% take 1: learning rate = .000387
% take 2: learning rate = .001
% take 3: learning rate = .0005
% pred = toy_forecast(net, XTest{1}, 100, 25, p, true);

% Save the output
% outputFile = fullfile("data/networks/full-nets/retrained_nets", 'test_net.mat');
% save(outputFile, 'net', 'info', 'error_vec', 'training_rmse_vec');

function error = validate_net(net, X_test, ns, k, p)
    error = 0;
    parfor i = 1:numel(X_test)
        pred = full_forecast_norecur(net, X_test{i}, ns(i), k, p);
%         size(pred)
        pred = pred(:,end-k+1:end);
        g_truth = X_test{i}(:,ns(i)+1:ns(i)+k);
        rmse = sqrt(immse(pred, single(g_truth)));
        error = error + rmse;
    end
    error = error/numel(X_test);
end

function plot_training(e_vec, e_vec_its, training_vec)
    clf
    plot(e_vec_its, e_vec)
    hold on
    plot(movmean(training_vec, 5))
    drawnow;
end

% function error = validate_pitch_only(net, X_test, ns, k, p, num_recur)
%     error = 0;
%     for i = 1:numel(X_test)
%         pred = full_forecast(net, X_test{i}, ns(i), k, p, num_recur, false);
% %         size(pred)
%         pred = pred(14,end-k+1:end);
%         g_truth = X_test{i}(14,ns(i)+1:ns(i)+k);
%         rmse = sqrt(immse(pred, single(g_truth)));
%         error = error + rmse;
%     end
%     error = error/numel(X_test);
% end