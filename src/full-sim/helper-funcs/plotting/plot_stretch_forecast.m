function plot_stretch_forecast(nets, data1, data2, n, k, pitch_idx, p)

close all
t = tiledlayout(2, 4);
cutoff = n+k*9+50;

time_steps = 1:cutoff;
time_stamps = time_steps/50;

mu = p.mu(pitch_idx);
sig = p.sig(pitch_idx);

% dark blue: '#332288'
% dark green: '#117733'
% light green: '#44AA99'
% light blue: '#88CCEE'
% yellow: '#DDCC77'
% pink: '#CC6677'
% wine: '#882255'
gt_color = '#88CCEE';
dashed1_color = '#332288';
dashed2_color = '#CC6677';

for sf = [2, 4, 6, 8]
    net = nets{sf};
    
    % Make prediction
    resetState(net);
    [new_net, Z] = predictAndUpdateState(net, [data1(:,1:end-k)]); %repmat(0.02, 1, size(data,2)-k)], "ExecutionEnvironment","auto");
    
    nexttile
    plot(time_stamps, data1(pitch_idx,1:cutoff)*sig+mu, 'Color', gt_color, 'LineWidth', 1)
    hold on
    plot(time_stamps(sf+1:n+sf), Z(1,1:n)*sig+mu, '-.', 'Color', dashed1_color, 'LineWidth', 2.)
    plot(time_stamps(n+sf:sf:n+sf*k), Z(:,n)*sig+mu, ':', 'Color', dashed2_color, 'LineWidth', 2.5)
    
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
    if sf == 2
        ylabel("Trajectory 1")
    end
end

for sf = [2, 4, 6, 8]
    net = nets{sf};
    
    % Make prediction
    resetState(net);
    [new_net, Z] = predictAndUpdateState(net, [data2(:,1:end-k)]); %repmat(0.02, 1, size(data,2)-k)], "ExecutionEnvironment","auto");
    
    nexttile
    plot(time_stamps, data2(pitch_idx,1:cutoff)*sig+mu, 'Color', gt_color, 'LineWidth', 1)
    hold on
    plot(time_stamps(sf+1:n+sf), Z(1,1:n)*sig+mu, '-.', 'Color', dashed1_color, 'LineWidth', 2.)
    plot(time_stamps(n+sf:sf:n+sf*k), Z(:,n)*sig+mu, ':', 'Color', dashed2_color, 'LineWidth', 2.5)
    
%     title(strcat(string(sf/2), "s Prediction"))
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
    if sf == 2 
        ylabel("Trajectory 2")
    end
end

%%
xlabel(t, "Simulation Time (s)");
ylabel(t, "Vehicle Pitch (rad)")
leg = legend("Ground Truth", "1 Step Prediction", "25 Step Prediction");
leg.Layout.Tile = 'south';
leg.Box = "on";
leg.Orientation = "horizontal";
% title(t, "Pitch Prediction Over Increasing Time Spans")

set(t, ...
    'TileSpacing', 'compact')

h=gcf;
set(h,'PaperOrientation','landscape');
set(h, 'PaperPositionMode', 'auto');
set(h, 'Position', [100 100 1000 425]);
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print(gcf, '-dpdf', 'data\plots\stretch_viz.pdf');

end