% Actual positions
chan_idxs.act_rpy = [1:3];
chan_idxs.act_xyz = [4:6];
chan_idxs.act_joint_pos = [7:10];
chan_idxs.act_pitch = 2;

% Actual velocities
chan_idxs.act_angular_vels = [11:13];
chan_idxs.act_linear_vels = [14:16];
chan_idxs.act_joint_vels = [17:20];

% Measured positions 
chan_idxs.meas_rpy = [21:23];
chan_idxs.meas_xyz = [24:26];
chan_idxs.meas_joint_pos = [27:30];

% Measured velocities
chan_idxs.meas_angular_vels = [31:33];
chan_idxs.meas_linear_vels = [34:36];
chan_idxs.meas_joint_vels = [37:40];

% Desired velocities
chan_idxs.des_joint_vels = [41:44];

outputFile = fullfile("data/full-sim-data-022223", 'channel_dict.mat');
save(outputFile, 'chan_idxs')
