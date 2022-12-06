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

load('C:\Users\OSU\Documents\GitHub\PitchPrediction\data\networks\full-nets\simple_w_stretch_factor\stretch_1_take_1.mat')

sf = 1;
k = 25; 

for n = 1:numel(XTest)
    resp = zeros(k, size(XTest{n}, 2)-sf*k);
    if size(XTest{n}, 2) > (sf+.1)*k
        for t = 1:size(XTest{n}, 2)-sf*k
            resp(:,t) = XTest{n}(pitch_idx, t+sf:sf:t+sf*k)';
        end
        Resp_Test{n} = resp;
        Inputs_Test{n} = XTest{n}(:,1:end-sf*k);
    end
end

%%
sum_rmses = 0;
for i = 1:numel(Inputs_Test)
    data = Inputs_Test{i};
    resetState(net);
    [net, Z] = predictAndUpdateState(net, data, "ExecutionEnvironment","auto");
    sum_rmses = sum_rmses + sqrt(immse(Z, single(Resp_Test{i})));
end
avg_rmse = sum_rmses/numel(Inputs_Test)