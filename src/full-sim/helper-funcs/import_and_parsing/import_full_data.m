% function train_toy_data()
%% Data import and processing
sequence_data = import_traj_no_orientation("data/full-sim-data/data-no-orientation");
orientation_data = import_orientations("data/full-sim-data/data-rpy");

%%
%{
X(:,1) is a column vector.
1-7: x, y, z, j_E, j_D, j_C, j_B (qs4,qs5,qs6,qs7,qs8,qs9,qs10)
8-14: vehicle velocities (twist, rpyxyz)(vs1,vs2,vs3,vs4,vs5,vs6)
15-18: joint angular velocities (vs7,vs8,vs9,vs10)
19-22: Desired joint velocities (des_vsE,des_vsD,des_vsC,des_vsB)
23-25: Vehicle orientation (RPY)

: time step of data collection (constant)
8-9: known starting positions (constant)
10-11: goal end positions (constant)
12-13: known starting velocities (constant)
14-15: goal end velocities (constant)
X(:,1:6) are fed to the LSTM, while X(:,7:end) are sent directly to the
FCN.
%}
waypoint_data = import_waypoint_data("data/full-sim-data/full-sim-waypoints_080622.csv");
for idx = 1:numel(sequence_data)
    combo_data{idx} = [sequence_data{idx}; orientation_data{idx}];
end

%%
nan_IDs = ID_nans(combo_data)
for idx = length(nan_IDs):-1:1
    combo_data(nan_IDs(idx)) = [];
    waypoint_data(:, nan_IDs(idx)) = [];
end

[combo_data, p] = normalize_data(combo_data);
bad_ids = ID_outliers(combo_data)

%%
% Remove outliers (from highest idx to lowest idx)
for idx = length(bad_ids):-1:1
    combo_data(bad_ids(idx)) = [];
    waypoint_data(:, bad_ids(idx)) = [];
end

%%
% Determine division between train and test data
numObservations = numel(combo_data);
idxTrain = 1:floor(0.95*numObservations);
idxTest = floor(0.95*numObservations)+1:numObservations;

% Split sequence data
seq_dataTrain = combo_data(idxTrain);
seq_dataTest = combo_data(idxTest);
wp_dataTrain = waypoint_data(:,idxTrain);
wp_dataTest = waypoint_data(:,idxTest);

%%

numChannels = size(seq_dataTrain{1}, 1);

for n = 1:numel(seq_dataTrain)
    X = seq_dataTrain{n};
    wp_array = repmat(wp_dataTrain(:,n), 1, length(X)-1);
    XTrain{n} = [X(:,1:end-1); wp_array];
    TTrain{n} = X(:,2:end);
end

for n = 1:numel(seq_dataTest)
    X = seq_dataTest{n};
    wp_array = repmat(wp_dataTest(:,n), 1, length(X)-1);
    XTest{n} = [X(:,1:end-1); wp_array];
    TTest{n} = X(:,2:end);
end

%% Sort by sequence length
for i=1:numel(XTrain)
    sequence = XTrain{i};
    sequenceLengths(i) = size(sequence,2);
end

[sequenceLengths,idx] = sort(sequenceLengths,'descend');
XTrain = XTrain(idx);
TTrain = TTrain(idx);

%% Save the output
outputFile = fullfile("data/full-data-matlab", 'FullData_081022.mat');
save(outputFile, 'XTest', 'TTest', 'XTrain', 'TTrain', 'p')


