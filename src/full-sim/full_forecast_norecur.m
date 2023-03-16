%{
Inputs: an RNN (net)
a set of ground truth training data (X) that can be input to net
a number of steps into the network to start forecasting (n)
a number of steps ahead to forecast (k)
the normalization parameters p (for un-normalizing the output)

Returns:
full_pred: the entire prediction, of which the first n steps are one-step
predictions from ground truth and n+1:n+k are autoregressive predictions.

Last modified 12/6/22
%}
function full_pred = full_forecast_norecur(net, X, n, k)
    % Reset the state for a new prediction
    net = resetState(net);
    num_chans = size(X, 1);

    % Predict and update until the forecasting start
    [net, Z] = predictAndUpdateState(net, X(:,1:n), "ExecutionEnvironment","auto");
    % Get the last output
    Xt = Z(:,end);
    % Setup forecast array
    Y = zeros(num_chans, k-1);

    % Generate the forecast
    for i = 1:k-1
        [net, outputs] = predictAndUpdateState(net, Xt, "ExecutionEnvironment", "auto");
        Y(:,i) = outputs;
        Xt = outputs;
    end

    % concatenate the single step and multi-step predictions
    full_pred = cat(2, Z, Y);    
end