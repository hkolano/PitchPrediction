chan_idxs.xyz_poses = [1:3];
chan_idxs.xyz_vels = [11:13];
chan_idxs.ry_poses = [22, 24];
chan_idxs.ry_vels = [8, 10];
chan_idxs.pitch = [23, 9];
chan_idxs.manip_poses = [4:7];
chan_idxs.manip_vels = [14:17];
chan_idxs.manip_des_vels = [18:21];
chan_idxs.dt = [25];
chan_idxs.goal_poses = [26:33];
chan_idxs.goal_vels = [34:41];

outputFile = fullfile("data/full-data-matlab", 'channel_dict.mat');
save(outputFile, 'chan_idxs')
