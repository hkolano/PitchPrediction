load("data/full-sim-data-091023/UnnormalizedTableTrainData.mat")
load("data/full-sim-data-091023/TrainData.mat")

for i = 1:numel(sequence_data)
    traj_table = sequence_data{i};
    for n = 1:width(traj_table)
        mu = p.mu(n);
        sig = p.sig(n);
        traj_table{:,n} = traj_table{:,n}-mu ./ sig;
    end
    sequence_data{i} = traj_table;
end

outputFile = fullfile("data/full-sim-data-091023", 'NormalizedTableTrainData.mat');
save(outputFile, 'sequence_data', 'p', '-v7.3')