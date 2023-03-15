%{
Plots a simple prediction and an autoregressive prediction over different
time scales. 

Last modified 2/28/23
%}

%% Data Loading
% Validation Set
load("data/full-sim-data-022223/FullData_50Hz.mat")
[sorted_XTest_50Hz, I] = sort_data_by_length(XTest)
%%
load("data/full-sim-data-022223/FullData_10Hz.mat")
sorted_XTest_10hz = XTest(flip(I, 2));

val_set = 'data/full-sim-data-022223/val_set.mat';
load(val_set)

%%
load('data/full-sim-data-022223/channel_dict.mat')

pitch_idx = chan_idxs.act_pitch;
elimd_gps = ["meas_xyz", "meas_joint_vels", "meas_linear_vels"];
all_idxs = get_remaining_idxs(elimd_gps, chan_idxs);


%%
k = 25;
numUnits = 384;

stretch_nets = {};
auto_nets = {};
stretch_inputs_14 = {};
stretch_inputs_15 = {};
stretch_responses = {};

%%
for stretch = 1:8
    [Inputs_Test_14, Resp_Test] = transform_data_for_stretch_study(sorted_XTest_50Hz, stretch, k, all_idxs, pitch_idx);
    [Inputs_Test_15, ~] = transform_data_for_stretch_study(sorted_XTest_50Hz, stretch, k, [20 all_idxs], pitch_idx);
    load(strcat("/nfs/stak/users/rosettem/PitchPrediction/data/networks/iros-nets/simple_w_stretch_factor/stretch_", string(stretch), "_take_1.mat"))
    %load(strcat('data\networks\icra-redo-nets\simple_w_stretch_factor\stretch_', string(stretch), '_take_2.mat'))
    stretch_nets{stretch} = net;
    stretch_inputs_14{stretch} = Inputs_Test_14;
    stretch_inputs_15{stretch} = Inputs_Test_15;
    stretch_responses{stretch} = Resp_Test;
end

%%
ks = [5 10 20 30 40];
for i = 1:length(ks)
    k = ks(i);
    load(strcat("data\networks\iros-nets\consolidated_autoreg\k", string(k), "\k", string(k), "_take1_50epochs.mat"))
    auto_nets{i} = net;
end
%%
val_num = 9;
traj_num = val_idxs(val_num);
n_50 = val_ns_50hz(val_num);
n_10 = val_ns_10hz(val_num)
this_XTest_10hz = sorted_XTest_10hz{traj_num};
this_XTest_50hz = sorted_XTest_50Hz{traj_num};

k=25;
cutoff = n_50+k*10;

time_steps = 1:cutoff;
time_stamps_50 = time_steps/50;
time_stamps_10 = time_steps/10;

mu = p.mu(pitch_idx);
sig = p.sig(pitch_idx);

gt_color = '#88CCEE';
dashed1_color = '#332288';
dashed2_color = '#CC6677';

close all
t = tiledlayout(2, 4);

for sf = [2, 4, 6, 8]
    net = stretch_nets{sf};
    inputs = stretch_inputs_15{sf}{traj_num};

    % Make prediction
    resetState(net);
    [new_net, Z] = predictAndUpdateState(net, [inputs(:,1:end-k)]); %repmat(0.02, 1, size(data,2)-k)], "ExecutionEnvironment","auto");
    
    nexttile
    plot(time_stamps_50, this_XTest_50hz(pitch_idx, 1:cutoff)*sig+mu, 'Color', gt_color, 'LineWidth', 1)
    hold on
    plot(time_stamps_50(sf+1:n_50+sf), Z(1,1:n_50)*sig+mu, '-.', 'Color', dashed1_color, 'LineWidth', 2.)
    plot(time_stamps_50(n_50+sf:sf:n_50+sf*k), Z(:,n_50)*sig+mu, ':', 'Color', dashed2_color, 'LineWidth', 2.5)
    
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
    k_auto = ks(auto_id);
    net=auto_nets{auto_id+1};
    cutoff_10hz = floor(cutoff/5);
    clear pred
    
    % Make prediction
    resetState(net);
    pred = full_forecast_norecur(net, this_XTest_10hz(all_idxs,:), n_10, k_auto);

    nexttile
    plot(time_stamps_10(1:cutoff_10hz), this_XTest_10hz(pitch_idx, 1:cutoff_10hz)*sig+mu, 'Color', gt_color, 'LineWidth', 1)
    hold on
    plot(time_stamps_10(2:n_10+1), pred(pitch_idx, 1:n_10)*sig+mu, '-.', 'Color', dashed1_color, 'LineWidth', 2.)
    plot(time_stamps_10(n_10+2:n_10+k_auto), pred(pitch_idx, n_10+1:end)*sig+mu, ':', 'Color', dashed2_color, 'LineWidth', 2.5)
    
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
filename = strcat('data\plots\iros-plots\stretch_viz_valn', string(val_num), '.fig');
print(gcf, '-dpdf', filename);
