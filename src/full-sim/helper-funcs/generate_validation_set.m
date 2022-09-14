% Define a validation set
% load('data/full-data-matlab/channel_subgroups/no_goal_poses/no_manip_vels/no_goal_vels/data_without_xyz_poses.mat')
load('data/full-data-matlab/FullData_17chan_10Hz.mat')
k = 40;

XTest = XTest_10hz(1:end-5);
XTrain = XTrain_10hz(1:end-35);
TTest = TTest_10hz(1:end-5);
TTrain = TTrain_10hz(1:end-35);

num_vals = 20;
val_idxs = [];
val_ns = [];
for i = 1:num_vals
    val_idxs(i) = randi(size(XTest,2));
    val_ns(i) = randi([2, size(XTest{val_idxs(i)}, 2)-k-1]);
end

outputFile = fullfile("data/full-data-matlab", 'val_set_post_abl_k40.mat');
save(outputFile, 'val_idxs', 'val_ns');