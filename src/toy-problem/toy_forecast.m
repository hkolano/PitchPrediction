%{
Inputs: an RNN (net)
a set of ground truth training data (X)
a number of steps into the network to start forecasting
a number of steps ahead to forecast (k)

Returns:

%}
function Y = toy_forecast(net, X, n, k, p, make_plot)
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
         
    if make_plot == true
        figure
        order = [1 4 2 5 3 6];
        t = tiledlayout(3,2, 'TileSpacing', 'Compact');
        ylabels = ["Vehicle Pitch", "Joint 1", "Joint 2", "", "", ""];
        titles = ["Joint Position (radians)", "Joint Velocity (rad/s)", "", "", "", ""];
        
        
        for i = 1:6
            idx = order(i);
            ax = nexttile;
            plot((1:size(X(1,:), 2))/50, X(idx,:).*p.sig(idx)+p.mu(idx), 'Color', '#FFC20A', 'LineWidth', 2)
            hold on
            plot((n+1:n+k+1)/50, Y(idx,:).*p.sig(idx)+p.mu(idx), '--', 'Color', '#0C7BDC', 'LineWidth', 2)
            xlabel("Time (s)")
            ylabel(ylabels(idx))
            xline(n/50, 'k-.')
            title(titles(i))
            legend("Ground Truth", "Forecast", "Prediction Start")
        end
        title(t, "Pitch Forecast based on Single Step Training")
    end
    
end