%% Data import and processing
sequence_data = import_trajs("data/full-sim-data-022223");

%%
%{
X(:,1) is a column vector.
1-10: Actual position data (qs)
11-20: Actual velocity data (vs)
21-30: Noisy position data (noisy_qs)
31-40: Noisy velocity data (noisy_vs)
41-44: Desired velocities 
      
Old information:
1-7: x, y, z, j_E, j_D, j_C, j_B (qs4,qs5,qs6,qs7,qs8,qs9,qs10)
8-14: vehicle velocities (twist, rpyxyz)(vs1,vs2,vs3,vs4,vs5,vs6)
15-18: joint angular velocities (vs7,vs8,vs9,vs10)
19-22: Desired joint velocities (des_vsE,des_vsD,des_vsC,des_vsB)
23-25: Vehicle orientation (RPY)
%}

%%
nan_IDs = ID_nans(sequence_data)
for idx = length(nan_IDs):-1:1
    sequence_data(nan_IDs(idx)) = [];
end

[sequence_data, p] = normalize_data(sequence_data);
bad_ids = ID_outliers(sequence_data)

%%
% Remove outliers (from highest idx to lowest idx)
for idx = length(bad_ids):-1:1
    sequence_data(bad_ids(idx)) = [];
end

%%
% Determine division between train and test data
numObservations = numel(sequence_data);
idxTrain = 1:floor(0.9*numObservations);
idxTest = floor(0.9*numObservations)+1:numObservations;

% Split sequence data
seq_dataTrain = sequence_data(idxTrain);
seq_dataTest = sequence_data(idxTest);

%%

numChannels = size(seq_dataTrain{1}, 1);

for n = 1:numel(seq_dataTrain)
    X = seq_dataTrain{n};
    XTrain{n} = X(:,1:end-1);
    TTrain{n} = X(:,2:end);
end

for n = 1:numel(seq_dataTest)
    X = seq_dataTest{n};
    XTest{n} = X(:,1:end-1);
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
outputFile = fullfile("data/full-sim-data-022223", 'FullData.mat');
save(outputFile, 'XTest', 'TTest', 'XTrain', 'TTrain', 'p')


