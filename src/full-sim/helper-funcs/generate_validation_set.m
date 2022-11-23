% Define a validation set
% load('data/full-data-matlab/channel_subgroups/no_goal_poses/no_manip_vels/no_goal_vels/data_without_xyz_poses.mat')
load('data/full-sim-data-110822/FullData_10Hz.mat')
k = 4;

% XTest = XTest_10hz(1:end-5);
% XTrain = XTrain_10hz(1:end-35);
% TTest = TTest_10hz(1:end-5);
% TTrain = TTrain_10hz(1:end-35);

XTest = XTest_10hz;
[~,I] = sort(cellfun(@length,XTest));
XTest = XTest(I);
XTest = XTest(3:end); % only use trajectories with lengths longer than 4s

num_vals = 250;
val_idxs = [];
val_ns = [];
for i = 1:num_vals
    val_idxs(i) = randi(numel(XTest));
    val_ns(i) = randi([2, size(XTest{val_idxs(i)}, 2)-10*k-1]);
end

outputFile = fullfile("data/full-sim-data-110822", 'val_set_10Hz.mat');
save(outputFile, 'val_idxs', 'val_ns');