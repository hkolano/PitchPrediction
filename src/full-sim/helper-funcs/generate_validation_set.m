% Define a validation set
load('data/full-data-matlab/channel_subgroups/no_goal_poses/no_manip_vels/no_goal_vels/data_without_xyz_poses.mat')

num_vals = 20;
val_idxs = [];
val_ns = [];
for i = 1:num_vals
    val_idxs(i) = randi(size(XTest,2));
    val_ns(i) = randi([2, size(XTest{val_idxs(i)}, 2)-k-1]);
end

outputFile = fullfile("data/full-data-matlab", 'val_set_post_abl.mat');
save(outputFile, 'val_idxs', 'val_ns');