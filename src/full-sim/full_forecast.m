%{
Inputs: an RNN (net)
a set of ground truth training data (X)
a number of steps into the network to start forecasting
a number of steps ahead to forecast (k)
the normalization parameters p (for un-normalizing the output)
make_plot: true/false whether to plot the results

Returns:

%}
function full_pred = full_forecast(net, X, n, k, p, num_rec_channels, make_plot)
    % Reset the state for a new prediction
    net = resetState(net);
    
    % Extract the constant values (waypoints, dt)
    const_vec = X(num_rec_channels+1:end,1);
    
    % Predict and update until the forecasting start
    [net, Z] = predictAndUpdateState(net, X(:,1:n), "ExecutionEnvironment","auto");
%     [net, Z] = predictAndUpdateState(net, X(:,1:n), "ExecutionEnvironment","cpu");
    
    % Get the last output
    Xt = [Z(:,end); const_vec];
    % Setup forecast array
    Y = zeros(num_rec_channels, k-1);
%     Y(:,1) = Z(:,end);
    
    % Generate the forecast
    for i = 1:k-1
        [net, outputs] = predictAndUpdateState(net, Xt, "ExecutionEnvironment", "auto");
        Y(:,i) = outputs;
        Xt = [outputs; const_vec];
    end
    
    full_pred = cat(2, Z, Y);
         
    if make_plot == true
        order = [22 8 23 9 24 10];
        t = tiledlayout(3,2, 'TileSpacing', 'Compact');
        ylabels = ["Roll", "", "Pitch", "",  "Yaw", ""];
        titles = ["R/P/Y(radians)", "Joint Velocity (rad/s)", "", "", "", ""];
              
        for i = 1:6
            idx = order(i);
            ax = nexttile;
            plot((1:size(X(1,:), 2))/50, X(idx,:).*p.sig(idx)+p.mu(idx), 'Color', '#FFC20A', 'LineWidth', 2)
            hold on
            plot((2:n+k)/50, full_pred(idx,:).*p.sig(idx)+p.mu(idx), '--', 'Color', '#0C7BDC', 'LineWidth', 2)
            xlabel("Time (s)")
            ylabel(ylabels(i))
            xline(n/50, 'k-.')
            title(titles(i))
            legend("Ground Truth", "Prediction", "Forecast Start")
        end
        title(t, "Pitch Forecast based on Single Step Training")
        set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
    end
    
end