%{ 
Defines a validation set to use for the stretch study. Saves 250 indicies
and "start points", for 10hz and for 50hz.

Last modified 12/6/22
%}
load('data/full-sim-data-110822/FullData_10Hz.mat')
k = 4; % minimum traj length in seconds

XTest = XTest_10hz;
[~,I] = sort(cellfun(@length,XTest));
XTest = XTest(I);
XTest = XTest(3:end); % only use trajectories with lengths longer than 4s

num_vals = 250;
val_idxs = zeros(1, num_vals);
val_ns_10hz = zeros(1, num_vals);
val_ns_50hz = zeros(1, num_vals);

for i = 1:num_vals
    val_idxs(i) = randi(numel(XTest));
    val_ns_10hz(i) = randi([2, size(XTest{val_idxs(i)}, 2)-10*k-1]);
    val_ns_50hz(i) = val_ns_10hz(i)*5;
end

outputFile = fullfile("data/full-sim-data-110822", 'val_set.mat');
save(outputFile, 'val_idxs', 'val_ns_10hz', 'val_ns_50hz');