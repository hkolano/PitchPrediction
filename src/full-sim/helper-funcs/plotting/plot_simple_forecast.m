function plot_simple_forecast(net, data, n, k)

% Make prediction
resetState(net)
[new_net, Z] = predictAndUpdateState(net, data(:,1:n), "ExecutionEnvironment","auto");

close all; 
plot(data(16,:), 'Color', '#DDCC77', 'LineWidth', 1)
hold on
plot(Z(1,:), '--', 'Color', '#0072B2', 'LineWidth', 2)
plot(n+1:n+k, Z(:,end), '--', 'Color', '#56B4E9', 'LineWidth', 2)

end