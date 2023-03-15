%{
Inputs:
net: autoregressive network to validate
X_test: a set of validation trajectories in val_idx order (not sorted)
ns: a list of indices to start forecasting
k: number of steps ahead to forecast (at 10hz)
pitch_idx: the index of the pitch AFTER feature groups have been eliminated

outputs: 
RMSE of the pitch over the k step forecast
%}

function pitch_error = validate_pitch_on_forecast_only(net, X_test, ns, k, all_idxs, pitch_idx)
    error = 0;
    num_long_enough_trajs = 0;
    for i = 1:250
        if ns(i) > 25
            % generate prediction
            resetState(net);
            pred = full_forecast_norecur(net, X_test{i}(all_idxs,:), ns(i), k);
            pred_length = size(pred, 2);
            forecast = pred(2,end-k+1:end);
    
            % recover ground truth for the forecast
            g_truth_forecast = X_test{i}(pitch_idx,pred_length-k+1:pred_length);
            
            % compare forecast to ground truth
            RMSE = rmse(forecast, single(g_truth_forecast));
            error = error + RMSE;
            num_long_enough_trajs = num_long_enough_trajs + 1;
        end
    end
    pitch_error = error/num_long_enough_trajs;
end