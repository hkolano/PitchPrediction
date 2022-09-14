load('data/full-data-matlab/FullData_18chan_50Hz.mat')

for i = 1:numel(XTrain)
    XTrain{i} = XTrain{i}(1:17,:);
end

for i = 1:numel(XTest)
    XTest{i} = XTest{i}(1:17,:);
end

outputfile = fullfile("data/full-data-matlab", "FullData_17chan_50Hz.mat");
save(outputfile, 'XTest', 'TTest', 'XTrain', 'TTrain', 'p', 'pitch_idx');

%%
load(fullfile("data/full-data-matlab/FullData_17chan_50Hz.mat"))

for i = 1:numel(XTrain)
    XTrain_25hz{i} = XTrain{i}(:,1:2:end);
    TTrain_25hz{i} = TTrain{i}(:,1:2:end);
end

for i = 1:numel(XTest)
    XTest_25hz{i} = XTest{i}(:,1:2:end);
    TTest_25hz{i} = TTest{i}(:,1:2:end);
end

outputfile25 = fullfile("data/full-data-matlab", "FullData_17chan_25Hz.mat");
save(outputfile25, 'XTrain_25hz', 'TTrain_25hz', 'XTest_25hz', 'TTest_25hz', 'p', 'pitch_idx');

for i = 1:numel(XTrain)
    XTrain_10hz{i} = XTrain{i}(:,1:5:end);
    TTrain_10hz{i} = TTrain{i}(:,1:5:end);
end

for i = 1:numel(XTest)
    XTest_10hz{i} = XTest{i}(:,1:5:end);
    TTest_10hz{i} = TTest{i}(:,1:5:end);
end

outputfile10 = fullfile("data/full-data-matlab", "FullData_17chan_10Hz.mat");
save(outputfile10, 'XTrain_10hz', 'TTrain_10hz', 'XTest_10hz', 'TTest_10hz', 'p', 'pitch_idx');
