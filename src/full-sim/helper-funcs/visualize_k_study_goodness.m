load('data/channel_dict.mat')
chan_idxs = rmfield(chan_idxs, 'pitch');
chan_idxs = rmfield(chan_idxs, 'dt');

elimd_gps = ["goal_poses", "manip_vels", "goal_vels"];
path = "data/full-data-matlab/channel_subgroups";
level_num = length(elimd_gps)+1;

for rnd_num = 1:length(elimd_gps)
    rnd_name = elimd_gps(rnd_num);
    chan_idxs = rmfield(chan_idxs, rnd_name);
    path = strcat(path, "/no_", elimd_gps(rnd_num));
end
fn = fieldnames(chan_idxs);

load(fullfile(path, "data_without_xyz_poses.mat"))
numChannels = size(XTrain{1}, 1);

% Initialize constants
% ks =[5, 10, 15, 20, 25, 30, 40, 50, 60, 70, 75, 80, 90, 100, 125, 150, 175, 200];
k = 25;
numUnits = 384;

load('C:\Users\hkola\Stuff\_ONR\RNNs for pitch prediction\PitchPrediction\data\networks\full-nets\simple_w_stretch_factor\stretch_1_take_1.mat')

plot_simple_forecast(net, XData{1}, 100, 25)