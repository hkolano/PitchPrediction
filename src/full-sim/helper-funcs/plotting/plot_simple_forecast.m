function plot_simple_forecast(net, data, n, k, pitch_idx, cutoff)

% Make prediction
resetState(net)
[new_net, Z] = predictAndUpdateState(net, [data(:,1:end-k)]); %repmat(0.02, 1, size(data,2)-k)], "ExecutionEnvironment","auto");

close all; 
plot(data(pitch_idx,1:cutoff), 'Color', '#DDCC77', 'LineWidth', 1)
hold on
plot(Z(1,2:n), '--', 'Color', '#0072B2', 'LineWidth', 2)
plot(n+1:n+k, Z(:,n), '--', 'Color', '#56B4E9', 'LineWidth', 2)

end
