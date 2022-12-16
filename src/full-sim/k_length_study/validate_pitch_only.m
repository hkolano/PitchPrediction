function pitch_error = validate_pitch_only(net, X_test, ns, k, p, pitch_idx)
    error = 0;
    for i = 1:numel(X_test)
        pred = full_forecast_norecur(net, X_test{i}, ns(i), k, p);
%         size(pred)
        pred = pred(pitch_idx,:);
        g_truth = X_test{i}(pitch_idx,2:length(pred)+1);
        rmse = sqrt(immse(pred, single(g_truth)));
        error = error + rmse;
    end
    pitch_error = error/numel(X_test);
end