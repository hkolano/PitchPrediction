%% One time setup
load("data/channel_dict.mat")
fields = {'pitch', 'dt'};
chan_idxs = rmfield(chan_idxs, fields);

elimd_gps = ["goal_poses", "manip_vels", "goal_vels", "xyz_poses"];
all_idxs = [1:25];
rnd_idxs = all_idxs;
for rnd_num = 1:length(elimd_gps)
    rnd_name = elimd_gps(rnd_num);
    rnd_idxs = setdiff(rnd_idxs, chan_idxs.(rnd_name));
    chan_idxs = rmfield(chan_idxs, rnd_name);
end
fn = fieldnames(chan_idxs);

pitch_idx = find(rnd_idxs == 23);
responses = rnd_idxs(rnd_idxs<25);

path = "data/full-sim-with-hydro/";
param_names = ["arm-added-mass", "arm-linear-drag", "vehicle-added-mass", "vehicle-linear-drag", "vehicle-quadratic-drag"];
percent_names = ["1percent", "10percent", "50percent"];

% sequence_data = import_traj_no_orientation(strcat(path, "single-model-1percent/", param, "/data-no-orientation-model", string(model_num)));
for percent_id = 1:length(percent_names)
    per_name = percent_names(percent_id);

    for param_id = 1:length(param_names)
        param = param_names(param_id);
    
        for model_num = 1:10
            load(strcat("data/full-sim-with-hydro/single-model-", per_name, "/", param, "/model", string(model_num), "_data.mat"))
            
            full_XData = XData;
            full_p = p;
            
            p.mu = full_p.mu([responses]);
            p.sig = full_p.sig([responses]);
            
            XData = downselect_data_subset(responses, full_XData);
            
            outputFile = fullfile(strcat("data/full-sim-with-hydro/single-model-", per_name, "/", param), strcat('model', string(model_num), '_data_18chan.mat'));
            save(outputFile, 'XData', 'p', 'pitch_idx')
        end
    end
end

