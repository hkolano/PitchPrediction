%{ 
Defines a validation set to use for the stretch study. Saves 250 indicies
and "start points", for 10hz and for 50hz.

Last modified 12/6/22
Last run 12/6/22
%}
load('data/full-sim-data-110822/FullData.mat')
load('data/full-sim-data-110822/FullData_10Hz.mat')
k = 4; % minimum traj length in seconds

%%
[sorted_XTest_50hz, I] = sort_data_by_length(XTest);
sorted_XTest = XTest_10hz(flip(I,2));

breakpoint = 0;

% Only use trajectories with lengths longer than 4.4s
for traj_num = 1:numel(sorted_XTest)
    if size(sorted_XTest{traj_num}, 2) < k*11
        breakpoint = traj_num;
        break
    end
end
sorted_XTest = sorted_XTest(1:breakpoint-1);

%%
num_vals = 250;
val_idxs = zeros(1, num_vals);
val_ns_10hz = zeros(1, num_vals);
val_ns_50hz = zeros(1, num_vals);

for i = 1:num_vals
    this_val_idx = randi(numel(sorted_XTest));
    this_traj_size = size(sorted_XTest{this_val_idx}, 2);
    last_possible_pred_start = this_traj_size-10*k-1;
    val_n_10hz = randi([2, last_possible_pred_start]);
%     fprintf('Trajectory length: %d ; prediction start %d \n', this_traj_size, val_n_10hz);

    val_idxs(i) = this_val_idx;
    val_ns_10hz(i) = val_n_10hz;
    val_ns_50hz(i) = val_n_10hz*5;
%     fprintf('50hz buffer: %d \n', size(sorted_XTest_50hz{this_val_idx}, 2)-val_ns_50hz(i))
end

outputFile = fullfile("data/full-sim-data-110822", 'val_set.mat');
save(outputFile, 'val_idxs', 'val_ns_10hz', 'val_ns_50hz');