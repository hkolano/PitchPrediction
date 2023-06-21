%{
Downsamples data from 50Hz to 10Hz, saves as 'XTrain_10hz', etc.

Used to generate FullData_10Hz.mat
Last modified 12/6/22
%}
load('data/full-sim-data-022223/FullData_50Hz.mat')

for i = 1:numel(XTrain)
    XTrain_lesshz{i} = XTrain{i}(:,1:5:end);
    TTrain_lesshz{i} = TTrain{i}(:,1:5:end);
end

for i = 1:numel(XTest)
    XTest_lesshz{i} = XTest{i}(:,1:5:end);
    TTest_lesshz{i} = TTest{i}(:,1:5:end);
end

XTrain = XTrain_lesshz;
TTrain = TTrain_lesshz;
XTest = XTest_lesshz;
TTest = TTest_lesshz;

outputfile10 = fullfile("data/full-sim-data-022223", "FullData_10Hz.mat");
save(outputfile10, 'XTrain', 'TTrain', 'XTest', 'TTest', 'p');
