%{
Inputs: an RNN (net)
a set of ground truth training data (X)
a number of steps into the network to start forecasting
a number of steps ahead to forecast (k)
the normalization parameters p (for un-normalizing the output)
make_plot: true/false whether to plot the results

Returns:

%}
function full_pred = full_forecast_no_recur(net, X, n, k)
    % Reset the state for a new prediction
    net = resetState(net);
    
    % Predict and update until the forecasting start
    [Z, net_state] = predict(net, X(:,1:n));
    net.State = net_state;
%     [net, Z] = predictAndUpdateState(net, X(:,1:n), "ExecutionEnvironment","cpu");
    
    % Get the last output
    Xt = Z(:,end);
    % Setup forecast array
    Y = zeros(17, k-1);
%     Y(:,1) = Z(:,end);
    
    % Generate the forecast
    for i = 1:k-1
        [outputs, net_state] = predict(net, Xt);
        Y(:,i) = outputs;
        Xt = outputs;
        net.State = net_state;
    end
    
    full_pred = cat(2, Z, Y);
         
%     if make_plot == true
% %         order = [22 8 23 9 24 10];
%         order = [13 4 14 5 15 6];
%         t = tiledlayout(3,2, 'TileSpacing', 'Compact');
%         ylabels = ["Roll", "", "Pitch", "",  "Yaw", ""];
%         titles = ["R/P/Y(radians)", "Joint Velocity (rad/s)", "", "", "", ""];
%               
%         for i = 1:6
%             idx = order(i);
%             ax = nexttile;
%             plot((1:size(X(1,:), 2))/50, X(idx,:).*p.sig(idx)+p.mu(idx), 'Color', '#FFC20A', 'LineWidth', 2)
%             hold on
%             plot((2:n+k)/50, full_pred(idx,:).*p.sig(idx)+p.mu(idx), '--', 'Color', '#0C7BDC', 'LineWidth', 2)
%             xlabel("Time (s)")
%             ylabel(ylabels(i))
%             xline(n/50, 'k-.')
%             title(titles(i))
%             legend("Ground Truth", "Prediction", "Forecast Start")
%         end
%         title(t, "Pitch Forecast based on Single Step Training")
%         set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
%     end
    
end