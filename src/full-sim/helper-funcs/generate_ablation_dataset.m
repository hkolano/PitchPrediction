load("data/full-data-matlab/FullData_081022.mat")
load("data/channel_dict.mat")

full_XTest = XTest;
full_XTrain = XTrain;
full_TTest = TTest;
full_TTrain = TTrain;
full_p = p;

fn = fieldnames(chan_idxs);
all_idxs = [1:41];

for gp_idx = 1:length(fn)
    keep_idxs = setdiff(all_idxs, chan_idxs.(fn{gp_idx}));
    pitch_idx = find(keep_idxs == 23);
    responses = keep_idxs(keep_idxs<25);

    p.mu = full_p.mu([responses]);
    p.sig = full_p.sig([responses]);

    XTest = downselect_data_subset(keep_idxs, full_XTest);
    TTest = downselect_data_subset(responses, full_TTest);
    XTrain = downselect_data_subset(keep_idxs, full_XTrain);
    TTrain = downselect_data_subset(responses, full_TTrain);

    for i = 1:numel(XTest)
        sequenceLengths(i) = size(XTest{i}, 2);
    end
    [sequenceLengths, idx] = sort(sequenceLengths, 'descend');
    XTest = XTest(idx);

    outputFile = fullfile("data/full-data-matlab/channel_subgroups", strcat('data_without_', fn{gp_idx}, '.mat'));
    save(outputFile, 'XTest', 'TTest', 'XTrain', 'TTrain', 'p', 'pitch_idx')

end
% keep_idxs = [4:10, 14:17, 18:21, 22:24, 25:41];
% keep_idxs_responses = [4:10, 14:17, 18:21, 22:24];
% 
% p.mu = p.mu([4:10, 14:17, 18:21, 22:24]);
% p.sig = p.sig([4:10, 14:17, 18:21, 22:24]);
% 
% XTest = downselect_data_subset(keep_idxs, XTest);
% TTest = downselect_data_subset(keep_idxs_responses, TTest);
% XTrain = downselect_data_subset(keep_idxs, XTrain);
% TTrain = downselect_data_subset(keep_idxs_responses, TTrain);
% 
% outputFile = fullfile("data/full-data-matlab", 'FullData_NoVehXYZ_081022.mat');
% save(outputFile, 'XTest', 'TTest', 'XTrain', 'TTrain', 'p')


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