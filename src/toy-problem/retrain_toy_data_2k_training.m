%% Import data
load('data/networks/toy-nets/retrained_071322_v3.mat')
load('data/toy-data-matlab/TestandTrainData_071322.mat')
k = 25;     % Number of time steps to forecast (0.5s)

retrain_options = trainingOptions("adam", ...
    MaxEpochs=1, ...
    MiniBatchSize=20, ...
    SequencePaddingDirection="right");

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

validate = @(net) validate_net(net, XTest, val_idxs, val_ns, k, p);


%% Train on predictions
error = validate(net)
error_vec = [error];
% pred = toy_forecast(net, XTest{1}, 100, 25, p, true);

for retrain_idx = 1:1000
    
    for it_num = 1:20
        traj_idx = randi(size(XTrain, 2));
        top_n_lim = size(XTrain{traj_idx}, 2) - k - 1;
        if top_n_lim <= k + 1
            n = top_n_lim;
        else
            n = randi([k, top_n_lim]);
        end
        
        % Generate a prediction
        pred = toy_forecast(net, XTrain{traj_idx}, n, k, p, false);
        if size(pred, 2) > 2*k 
            pred = pred(:,end-2*k+1:end);
        end

        wp_array = repmat(XTrain{traj_idx}(7:end,1), 1, length(pred));
        preds{it_num} = [pred; wp_array];
        if size(pred, 2) >= 2*k
            g_truth{it_num} = XTrain{traj_idx}(1:6,n-k+1:n+k);      
        else
            g_truth{it_num} = XTrain{traj_idx}(1:6,2:n+k);
        end
    end
    
    net = trainNetwork(preds, g_truth, layerGraph(net), retrain_options);
    
    if rem(retrain_idx, 25) == 0
        error = validate(net)
        disp(retrain_idx)
        error_vec = [error_vec error];
    end
    
end

% pred = toy_forecast(net, XTest{1}, 100, 25, p, true);

% Save the output
% outputFile = fullfile("data/networks/toy-nets", 'retrained_071522_v3_2.mat');
% save(outputFile, 'net', 'error_vec');

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