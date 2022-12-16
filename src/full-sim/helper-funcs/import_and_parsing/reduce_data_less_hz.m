%{
Downsamples data from 50Hz to 10Hz, saves as 'XTrain_10hz', etc.

Used to generate FullData_10Hz.mat
Last modified 12/6/22
%}
load('data/full-sim-data-110822/FullData.mat')

for i = 1:numel(XTrain)
    XTrain_10hz{i} = XTrain{i}(:,1:5:end);
    TTrain_10hz{i} = TTrain{i}(:,1:5:end);
end

for i = 1:numel(XTest)
    XTest_10hz{i} = XTest{i}(:,1:5:end);
    TTest_10hz{i} = TTest{i}(:,1:5:end);
end

outputfile10 = fullfile("data/full-sim-data-110822", "FullData_10Hz.mat");
save(outputfile10, 'XTrain_10hz', 'TTrain_10hz', 'XTest_10hz', 'TTest_10hz', 'p');
