load("data/full-data-matlab/FullData_18chan_50Hz.mat")
load('data/channel_dict.mat')
val_set = 'data/full-data-matlab/val_set_final_50hz.mat';

%%

numChannels = size(XTrain{1}, 1);
k = 25;
stretches = [1, 2, 3, 4, 5, 6, 7, 8];
stretch_forecast_errors2 = zeros(1,8);

for idx = 1:length(stretches)
    load(val_set)
    sf = stretches(idx);
    for n = 1:numel(XTest)
        resp = zeros(k, size(XTest{n}, 2)-sf*k);
        if size(XTest{n}, 2) > (sf+.1)*k
            for t = 1:size(XTest{n}, 2)-sf*k
                resp(:,t) = XTest{n}(pitch_idx, t+sf:sf:t+sf*k)';
            end
            Resp_Test{n} = resp;
            Inputs_Test{n} = XTest{n}(:,1:end-sf*k);
        end
    end

    XTest_subset = Inputs_Test(val_idxs);
    Gtruth_subset = Resp_Test(val_idxs);
    val_ns = floor(val_ns./sf);
    load(strcat("data\networks\full-nets\simple_w_stretch_factor\stretch_", string(sf), "_take_2.mat"))
    pitch_error = validate_pitch_on_stretch_forecast(net, XTest_subset, Gtruth_subset, val_ns, k, p, pitch_idx)
    stretch_forecast_errors2(idx) = pitch_error;

    load(strcat("data\networks\full-nets\simple_w_stretch_factor\stretch_", string(sf), "_take_3.mat"))
    pitch_error = validate_pitch_on_stretch_forecast(net, XTest_subset, Gtruth_subset, val_ns, k, p, pitch_idx)
    stretch_forecast_errors3(idx) = pitch_error;
end


function pitch_error = validate_pitch_on_stretch(net, X_test, G_truth, ns, k, p, pitch_idx)
    error = 0;
    for i = 1:numel(X_test)
        resetState(net);
        [net, pred] = predictAndUpdateState(net, X_test{i}(:,1:ns(i)));
%         size(pred)
        pred_onestep = pred(1,:);
        pred_forecast = pred(2:end, end);
        total_pitch_pred = [pred_onestep,pred_forecast'];
        g_truth = X_test{i}(pitch_idx,2:length(total_pitch_pred)+1);
        rmse = sqrt(immse(total_pitch_pred, single(g_truth)));
        error = error + rmse;
    end
    pitch_error = error/numel(X_test);
end

function pitch_error = validate_pitch_on_stretch_forecast(net, X_test, G_truth, ns, k, p, pitch_idx)
    error = 0;
    for i = 1:numel(X_test)
        resetState(net);
        [net, pred] = predictAndUpdateState(net, X_test{i}(:,1:ns(i)));
%         size(pred)
%         pred_onestep = pred(1,:);
        pred_forecast = pred(:, end);
%         total_pitch_pred = [pred_onestep,pred_forecast'];
        g_truth = G_truth{i}(:,ns(i));
        rmse = sqrt(immse(pred_forecast, single(g_truth)));
        error = error + rmse;
    end
    pitch_error = error/numel(X_test);
end