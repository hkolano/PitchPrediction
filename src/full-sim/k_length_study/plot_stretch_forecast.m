function plot_stretch_forecast(s_nets, a_nets, data_50hz, data_10hz, n, all_idxs, pitch_idx, p)

close all
t = tiledlayout(2, 4);
cutoff = n+25*9+50;
k = 25;

time_steps = 1:cutoff;
time_stamps_50 = time_steps/50;
time_stamps_10 = time_steps/10;

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
    net = s_nets{sf};
    [Inputs_Test, Resp_Test] = transform_data_for_stretch_study(data_50hz, sf, 25, all_idxs, pitch_idx);
    
    % Make prediction
    resetState(net);
    [new_net, Z] = predictAndUpdateState(net, [data_50hz(:,1:end-k)]); %repmat(0.02, 1, size(data,2)-k)], "ExecutionEnvironment","auto");
    
    nexttile
    plot(time_stamps_50, data_50hz(pitch_idx,1:cutoff)*sig+mu, 'Color', gt_color, 'LineWidth', 1)
    hold on
    plot(time_stamps_50(sf+1:n+sf), Z(1,1:n)*sig+mu, '-.', 'Color', dashed1_color, 'LineWidth', 2.)
    plot(time_stamps_50(n+sf:sf:n+sf*k), Z(:,n)*sig+mu, ':', 'Color', dashed2_color, 'LineWidth', 2.5)
    
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
        ylabel("Pitch Only")
    end
end

ks = [10 20 30 40];
for auto_id = 1:4
    k = ks(auto_id);
    net=a_nets{auto_id+1};
    cutoff_10hz = floor(cutoff/5);
    n_10hz = floor(n/5);
    clear pred
    
    % Make prediction
    resetState(net);
    pred = full_forecast_norecur(net, data_10hz, n_10hz, k);


    nexttile
    plot(time_stamps_10(1:cutoff_10hz), data_10hz(adj_pitch_idx, 1:cutoff_10hz)*sig+mu, 'Color', gt_color, 'LineWidth', 1)
    hold on
    plot(time_stamps_10(2:n_10hz+1), pred(adj_pitch_idx, 1:n_10hz)*sig+mu, '-.', 'Color', dashed1_color, 'LineWidth', 2.)
    plot(time_stamps_10(n_10hz+2:n_10hz+k), pred(adj_pitch_idx, n_10hz+1:end)*sig+mu, ':', 'Color', dashed2_color, 'LineWidth', 2.5)
    
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
    if auto_id == 1
        ylabel("Autoregressive")
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
set(h, 'Position', [100 100 1000 300]);
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print(gcf, '-dpdf', 'data\plots\stretch_viz.pdf');

end