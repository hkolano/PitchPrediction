%{
Inputs: an RNN (net)
a set of ground truth training data (X)
a number of steps into the network to start forecasting
a number of steps ahead to forecast (k)
the normalization parameters p (for un-normalizing the output)
make_plot: true/false whether to plot the results

Returns:

%}
function full_pred = full_forecast_norecur(net, X, n, k, p)
    % Reset the state for a new prediction
    net = resetState(net);

    num_chans = size(X, 1);
%     disp(strcat("Starting prediction of length ", string(n)))
%     start_pred = tic;
    % Predict and update until the forecasting start
    [net, Z] = predictAndUpdateState(net, X(:,1:n), "ExecutionEnvironment","auto");
%     [net, Z] = predictAndUpdateState(net, X(:,1:n), "ExecutionEnvironment","cpu");
%     toc(start_pred)
    % Get the last output
    Xt = Z(:,end);
    % Setup forecast array
    Y = zeros(num_chans, k-1);
%     Y(:,1) = Z(:,end);
    
%     disp(strcat("Starting forecast of length ", string(k-1)))
%     start_forecast = tic;
    % Generate the forecast
    for i = 1:k-1
        [net, outputs] = predictAndUpdateState(net, Xt, "ExecutionEnvironment", "auto");
        Y(:,i) = outputs;
        Xt = outputs;
    end
%     toc(start_forecast)
    full_pred = cat(2, Z, Y);
    
end