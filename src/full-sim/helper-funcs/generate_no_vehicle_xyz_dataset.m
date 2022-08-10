load("data/full-data-matlab/FullData_081022.mat")

keep_idxs = [4:10, 14:17, 18:21, 22:24, 25:41];
keep_idxs_responses = [4:10, 14:17, 18:21, 22:24];

p.mu = p.mu([4:10, 14:17, 18:21, 22:24]);
p.sig = p.sig([4:10, 14:17, 18:21, 22:24]);

XTest = downselect_data_subset(keep_idxs, XTest);
TTest = downselect_data_subset(keep_idxs_responses, TTest);
XTrain = downselect_data_subset(keep_idxs, XTrain);
TTrain = downselect_data_subset(keep_idxs_responses, TTrain);

outputFile = fullfile("data/full-data-matlab", 'FullData_NoVehXYZ_081022.mat');
save(outputFile, 'XTest', 'TTest', 'XTrain', 'TTrain', 'p')


%{
X(:,1) is a column vector.
1-7: x, y, z, j_E, j_D, j_C, j_B (qs4,qs5,qs6,qs7,qs8,qs9,qs10)
8-13: vehicle velocities (twist, rpyxyz)(vs1,vs2,vs3,vs4,vs5,vs6)
14-17: joint angular velocities (vs7,vs8,vs9,vs10)
18-21: Desired joint velocities (des_vsE,des_vsD,des_vsC,des_vsB)
22-24: Vehicle orientation (RPY)

: time step of data collection (constant)
8-9: known starting positions (constant)
10-11: goal end positions (constant)
12-13: known starting velocities (constant)
14-15: goal end velocities (constant)
X(:,1:6) are fed to the LSTM, while X(:,7:end) are sent directly to the
FCN.
%}