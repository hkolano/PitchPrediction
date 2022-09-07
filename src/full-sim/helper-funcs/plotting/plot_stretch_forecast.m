function plot_stretch_forecast(nets, data, n, k, pitch_idx, p)

close all
t = tiledlayout(2, 3);
cutoff = n+k*6+50;

time_steps = 1:cutoff;
time_stamps = time_steps/50;

mu = p.mu(pitch_idx);
sig = p.sig(pitch_idx);

for sf = 1:6
    net = nets{sf};
    
    % Make prediction
    resetState(net);
    [new_net, Z] = predictAndUpdateState(net, [data(:,1:end-k)]); %repmat(0.02, 1, size(data,2)-k)], "ExecutionEnvironment","auto");
    
    nexttile
    plot(time_stamps, data(pitch_idx,1:cutoff)*sig+mu, 'Color', '#DDCC77', 'LineWidth', 1)
    hold on
    plot(time_stamps(sf+1:n+sf), Z(1,1:n)*sig+mu, '--', 'Color', '#0072B2', 'LineWidth', 2)
    plot(time_stamps(n+sf:sf:n+sf*k), Z(:,n)*sig+mu, '--', 'Color', '#56B4E9', 'LineWidth', 2)
    
    title(strcat(string(sf/2), "s Prediction"))
    set(gca, ...
      'Box'         , 'off'     , ...
      'TickDir'     , 'out'     , ...
      'TickLength'  , [.02 .02] , ...
      'XMinorTick'  , 'on'      , ...
      'YMinorTick'  , 'on'      , ...
      'YGrid'       , 'on'      , ...
      'XColor'      , [.3 .3 .3], ...
      'YColor'      , [.3 .3 .3], ...
      'LineWidth'   , 1         );
end

xlabel(t, "Simulation Time (s)");
ylabel(t, "Vehicle Pitch (rad)")
leg = legend("Ground Truth", "Single Step Prediction", "k=25 Prediction");
leg.Layout.Tile = 'south';
leg.Box = "on";
leg.Orientation = "horizontal";
title(t, "Pitch Prediction Over Increasing Time Spans")

set(t, ...
    'TileSpacing', 'compact')

end