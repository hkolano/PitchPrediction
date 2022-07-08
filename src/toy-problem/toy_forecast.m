%{
Inputs: an RNN (net)
a set of ground truth training data (X)
a number of steps into the network to start forecasting
a number of steps ahead to forecast (k)

Returns:

%}
function Y = toy_forecast(net, X, n, k)
    % Reset the state for a new prediction
    net = resetState(net);
    
    % Extract the constant values (waypoints, dt)
    const_vec = X(7:end,1);
    
    % Predict and update until the forecasting start
    [net, Z] = predictAndUpdateState(net, X(:,1:n));
    
    % Get the last output
    Xt = [Z(:,end); const_vec];
    % Setup forecast array
    Y = zeros(6, k);
    Y(:,1) = Z(:,end);
    
    % Generate the forecast
    for i = 1:k
        [net, outputs] = predictAndUpdateState(net, Xt);
        Y(:,i+1) = outputs;
        Xt = [outputs; const_vec];
    end
         
    figure
    plot(X(1,:), 'g')
    hold on
    plot(n+1:n+k+1, Y(1,:), 'm--')
    xlabel("Time Step")
    xline(n, 'k-.')
    legend("Ground Truth", "Forecasted", "Prediction Start")    
    
end