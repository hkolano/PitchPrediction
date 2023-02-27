%{ 
Defines a validation set to use for the stretch study. Saves 250 indicies
and "start points", for 10hz and for 50hz.

Last modified 2/27/22
Last run 2/27/22
%}
load('data/full-sim-data-022223/FullData_50Hz.mat')
XTrain_50Hz = XTrain; 
XTest_50Hz = XTest;
TTrain_50Hz = TTrain;
TTest_50Hz = TTest;
load('data/full-sim-data-022223/FullData_10Hz.mat')
XTrain_10Hz = XTrain; 
XTest_10Hz = XTest;
TTrain_10Hz = TTrain;
TTest_10Hz = TTest;

clear XTrain XTest TTrain TTest 
k = 4; % minimum traj length in seconds

%%
[sorted_XTest_50hz, I] = sort_data_by_length(XTest_50Hz);
sorted_XTest_10Hz = XTest_10Hz(flip(I,2));

breakpoint = 0;

% Only use trajectories with lengths longer than 4.4s
for traj_num = 1:numel(sorted_XTest_10Hz)
    if size(sorted_XTest_10Hz{traj_num}, 2) < k*11
        breakpoint = traj_num;
        break
    end
end
if breakpoint ~= 0
    sorted_XTest_10Hz = sorted_XTest_10Hz(1:breakpoint-1);
end

%%
num_vals = 250;
val_idxs = zeros(1, num_vals);
val_ns_10hz = zeros(1, num_vals);
val_ns_50hz = zeros(1, num_vals);

for i = 1:num_vals
    this_val_idx = randi(numel(sorted_XTest_10Hz));
    this_traj_size = size(sorted_XTest_10Hz{this_val_idx}, 2);
    last_possible_pred_start = this_traj_size-10*k-1;
    val_n_10hz = randi([2, last_possible_pred_start]);
%     fprintf('Trajectory length: %d ; prediction start %d \n', this_traj_size, val_n_10hz);

    val_idxs(i) = this_val_idx;
    val_ns_10hz(i) = val_n_10hz;
    val_ns_50hz(i) = val_n_10hz*5;
%     fprintf('50hz buffer: %d \n', size(sorted_XTest_50hz{this_val_idx}, 2)-val_ns_50hz(i))
end

outputFile = fullfile("data/full-sim-data-022223", 'val_set.mat');
save(outputFile, 'val_idxs', 'val_ns_10hz', 'val_ns_50hz');