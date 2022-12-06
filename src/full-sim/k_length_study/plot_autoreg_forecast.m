function plot_autoreg_forecast(net, data, n, k, p, pitch_idx, cutoff)

figure

pred = full_forecast_norecur(net, data, n, k, p);
plot(data(pitch_idx,1:cutoff), 'Color', '#DDCC77', 'LineWidth', 1)
hold on
plot(2:n+1, pred(pitch_idx,1:n),'--','Color', '#0072B2', 'LineWidth', 2)
plot(n+1:n+k-1, pred(pitch_idx,n+1:n+k-1), '--', 'Color', '#56B4E9', 'LineWidth', 2)

end