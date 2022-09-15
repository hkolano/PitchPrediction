function pitch_error = validate_pitch_on_forecast_only(net, X_test, ns, k, p, pitch_idx)
    error = 0;
    for i = 1:numel(X_test)
        traj_size = (size(X_test{i}));
        pred = full_forecast_norecur(net, X_test{i}, ns(i), k, p);
        pred_size = size(pred);
        if pred_size <= traj_size
            g_truth = X_test{i}(pitch_idx,length(pred)-k+1:length(pred));
            pred = pred(pitch_idx,end-k+1:end);
            
            rmse = sqrt(immse(pred, single(g_truth)));
        else
            rmse = 0;
            disp("prediction exceeds trajectory length. Ignoring.")
        end
        error = error + rmse;
    end
    pitch_error = error/numel(X_test);
end