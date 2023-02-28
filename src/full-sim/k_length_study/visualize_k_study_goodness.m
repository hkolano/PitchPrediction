%{
Plots a simple prediction and an autoregressive prediction over different
time scales. 

Last modified 12/9/22
%}

%% Data Loading
% Validation Set
val_set = 'data/full-sim-data-110822/val_set.mat';
load(val_set)

load("data/full-sim-data-110822/FullData.mat")
load('data/full-sim-data-110822/FullData_10Hz.mat')

[sorted_XTest_50hz, I] = sort_data_by_length(XTest);
sorted_XTest_10hz = XTest_10hz(flip(I,2));

for n = 1:numel(sorted_XTest_10hz)
    sorted_XTest_10hz{n} = sorted_XTest_10hz{n}(all_idxs, :);
end

elimd_gps = ["xyz_poses", "xyz_vels", "goal_poses", "manip_des_vels", "goal_vels"];
all_idxs = get_remaining_idxs(elimd_gps);

%%
k = 25;
numUnits = 384;
pitch_idx = 23;

stretch_nets = {};
auto_nets = {};

for stretch = 1:8
    [Inputs_Test, Resp_Test] = transform_data_for_stretch_study(sorted_XTest_50hz, stretch, k, all_idxs, pitch_idx);
    load(strcat('data\networks\icra-redo-nets\simple_w_stretch_factor\stretch_', string(stretch), '_take_2.mat'))
    stretch_nets{stretch} = net;
end

ks = [5 10 20 30 40];
for i = 1:length(ks)
    k = ks(i);
    load(strcat("data\networks\icra-redo-nets\10Hz_k", string(k), "\take1_50epochs.mat"))
    auto_nets{i} = net;
end
%%

val_num = 18;
traj_num = val_idxs(val_num);
this_n = val_ns_50hz(val_num);
plot_stretch_forecast(stretch_nets, auto_nets, Inputs_Test{traj_num}, sorted_XTest_10hz{traj_num}, this_n, 25, 23, 13, p)
