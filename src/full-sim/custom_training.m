%% Load data
load('data/full-data-matlab/channel_subgroups/no_goal_poses/no_manip_vels/no_goal_vels/data_without_xyz_poses.mat')
for n = 1:numel(XTrain)
    XTrain{n} = XTrain{n}(1:end-1,:);
end

for n = 1:numel(XTest)
    XTest{n} = XTest{n}(1:end-1,:);
end

XTrain = XTrain';
XTest = XTest';
disp("Data loaded.")

%%
dsXTrain = arrayDatastore( XTrain, 'IterationDimension', 1, 'OutputType', 'same');
dsTTrain = arrayDatastore( TTrain, 'IterationDimension', 1, 'OutputType', 'same');
dsTrain = combine(dsXTrain, dsTTrain);

mbqTrain = minibatchqueue(  dsXTrain, 1, ...
                          'MiniBatchSize', 16, ...
                          'PartialMiniBatch', 'discard', ...
                          'MiniBatchFcn', @concatSequenceData, ...
                          'MiniBatchFormat', 'SCT', ...
                          'OutputAsDlarray', 1);

disp("Minibatches created.")

%% Set up network
layers = setup_abl_residual_gru_no_output(17);
net = dlnetwork(layers);

numEpochs = 10;
miniBatchSize = 16; 
initialLearnRate = 0.001; 

figure
C = colororder;
lineLossTrain = animatedline(Color=C(2,:));
ylim([0 inf])
xlabel("Iteration")
ylabel("Loss")
grid on

disp("Network set up.")

%% Iterate
for iteration = 1:10
    
    traj_indices = zeros(1, 16);
    traj_inputs = {};
    traj_outputs = {};

    for it_num = 1:miniBatchSize
        traj_idx = randi(size(XTrain, 2));
        traj_indices(it_num) = traj_idx;
        traj_inputs{it_num} = XTrain{traj_idx};
        traj_outputs{it_num} = TTrain{traj_idx};
    end

    [loss,gradients,state] = dlfeval(@modelLoss,net, traj_inputs,traj_outputs);

end

%% Functions
function [loss, gradients, state] = modelLoss(net, X, T)

% Forward data through network
[Y, state] = forward(net, X);

% Calculate the loss
loss = immse(Y, T);

% Calculate gradients wrt learnable parameters
gradients = dlgradient(loss, net.Learnables);

end

function x = concatSequenceData(x)
x = padsequences(x,2);
% y = padsequences(y, 2);
% y = onehotencode(cat(2,y{:}),1);
end



%%
% numChannels = size(XTrain{1}, 1);
% numRecChannels = size(TTrain{1}, 1);
% layers = setup_full_residual_gru(numChannels, numRecChannels);
% 

% 
% mbq = minibatchqueue(XTrain, ...
%     MiniBatchSize = miniBatchSize);
% 
% %%
% %     MaxEpochs=5, ...
% init_options = trainingOptions("adam", ...
%     MaxEpochs=100, ...
%     MiniBatchSize=16, ...
%     InitialLearnRate=0.001, ...
%     LearnRateDropPeriod = 5,...
%     LearnRateSchedule= 'piecewise', ...
%     LearnRateDropFactor= .9, ...
%     SequencePaddingDirection="right", ...
%     Plots="training-progress", ...
%     Shuffle='every-epoch', ...
%     ValidationData={XTest, TTest}, ...
%     ValidationFrequency = 60, ...
%     OutputNetwork='best-validation-loss');
% [net, info] = trainNetwork(XTrain,TTrain,layers,init_options);
% %     
% outputFile = fullfile("data/networks/full-nets", 'SingleStepNet_18chan_384units.mat');
% save(outputFile, 'net', 'info');