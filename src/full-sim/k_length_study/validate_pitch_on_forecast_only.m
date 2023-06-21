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

function pitch_error = validate_pitch_on_forecast_only(net, X_test, ns, k, input_idxs, pitch_idx)
    error = 0;
    for i = 1:numel(X_test)
        
        % generate prediction
        pred = full_forecast_norecur(net, X_test{i}(input_idxs,:), ns(i), k);
        pred_length = size(pred, 2);
        % TODO: If the second value is not pitch, change the 2 below
        forecast = pred(2, end-k+1:end);

        % recover ground truth for the forecast
        g_truth_forecast = X_test{i}(pitch_idx,pred_length-k+2:pred_length+1);
        
        % compare forecast to ground truth
        rmse = sqrt(immse(forecast, single(g_truth_forecast)));
        error = error + rmse;
    end
    pitch_error = error/numel(X_test);
end