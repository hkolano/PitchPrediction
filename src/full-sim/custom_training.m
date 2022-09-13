%% Load data
load('data/full-data-matlab/channel_subgroups/no_goal_poses/no_manip_vels/no_goal_vels/data_without_xyz_poses.mat')
for n = 1:numel(XTrain)
    XTrain{n} = XTrain{n}(1:end-1,:);
end

for n = 1:numel(XTest)
    XTest{n} = XTest{n}(1:end-1,:);
end

XTrain = XTrain';
TTrain = TTrain';
XTest = XTest';
TTest = TTest';
disp("Data loaded.")

%%
load('data/full-data-matlab/val_set_post_abl.mat')
XTest_subset = XTest(val_idxs);
TTest_subset = TTest(val_idxs);
XTest_subset_trimmed = trim_XData_to_n(XTest_subset, val_ns);
TTest_subset_trimmed = trim_YData_to_n_plus_k(TTest_subset, val_ns, 25);
dsXTrain = arrayDatastore( XTrain, 'IterationDimension', 1, 'OutputType', 'same');
dsTTrain = arrayDatastore( TTrain, 'IterationDimension', 1, 'OutputType', 'same');
dsTrain = combine(dsXTrain, dsTTrain);

dsXTest = arrayDatastore(XTest_subset_trimmed, 'IterationDimension', 1, 'OutputType', 'same');
dsTTest = arrayDatastore(TTest_subset_trimmed, 'IterationDimension', 1, 'OutputType','same');
dsTest = combine(dsXTest, dsTTest);

mbqTrain = minibatchqueue(  dsTrain, 3, ...
                          'MiniBatchSize', 16, ...
                          'PartialMiniBatch', 'discard', ...
                          'MiniBatchFcn', @concatSequenceData, ...
                          'MiniBatchFormat', {'CTB', 'CTB', 'CTB'}, ...
                          'OutputAsDlarray', [1, 1, 1]);

mbqVal = minibatchqueue(dsTest, 3, ...
                        "MiniBatchSize", 16, ...
                        "MiniBatchFcn", @concatSequenceData, ...
                        "MiniBatchFormat", {'CTB', 'CTB', 'CTB'}, ...
                        'OutputAsDlarray', [1, 1, 1]);

disp("Minibatches created.")

%% Set up network
% layers = setup_abl_residual_gru_no_output(17);
% net = dlnetwork(layers);
clear net
executionEnvironment = "gpu";

k = 2;

numEpochs = 2;
miniBatchSize = 16; 
initialLearnRate = 0.001;
save_freq = 5; % epochs

validate = @(net) validate_net(net, mbqVal, val_ns, k, miniBatchSize);
forecast_for_training = @(net, X, ns) minibatch_forecast(net, X, ns, k, miniBatchSize, true);

figure
C = colororder;
lineLossTrain = animatedline(Color=C(2,:));
lineLossVal = animatedline(Color=C(3,:));
ylim([0 inf])
xlabel("Iteration")
ylabel("Loss")
grid on

load("data/networks/full-nets/SingleStepNet_17chan_1epoch.mat")
disp("Network set up.")

%% Iterate
iteration = 0;
start = tic;
averageGrad = [];
averageSqGrad = [];

first_error = validate(net)
%%

for epoch = 1:numEpochs
    
    % Shuffle data
    shuffle(mbqTrain);

    % Loop over mini-batches
    while hasdata(mbqTrain)
%     for ct = 1:10
        iteration = iteration + 1;

        % read mini-batch of data
        [X, Y, mask] = next(mbqTrain);

        if canUseGPU
            X = gpuArray(X);
            Y = gpuArray(Y);
        end

        [loss,gradients,state] = dlfeval(@modelLoss,net, X,Y, mask, k, miniBatchSize, forecast_for_training);
        net.State = state;

        [net,averageGrad,averageSqGrad] = adamupdate(net,gradients,averageGrad,averageSqGrad,iteration, .001);
        
        D = duration(0,0,toc(start),'Format',"hh:mm:ss");
        loss = double(loss);
        addpoints(lineLossTrain,iteration,loss)
        title("Epoch: " + epoch + ", Elapsed: " + string(D))
        drawnow

        if rem(iteration, 10) == 0
            disp(iteration)
            error = validate_net(net, mbqVal, val_ns, k, miniBatchSize)
        end
    end
end

disp("Reached end of training.")

%% Functions
function [loss, gradients, state] = modelLoss(net, X, T, mask, k, mbs, custom_forward)

    ns = zeros(mbs, 1);
    for i = 1:mbs
        len = extractdata(sum(mask(1,i,:)));
        ns(i) = randi([2, len-k-1]);
    end
    [Z_dl, state] = custom_forward(net, X, ns);
    % Forward data through network
%     [Z, state] = forward(net, X);
   
    % Calculate the loss
    loss = 0;
    for i = 1:mbs
        loss = loss + ns(i)*mse(Z_dl(:,i,1:ns(i)+k-1), T(:,i,1:ns(i)+k-1));
    end
    loss = loss/sum(ns, 'all');
%     loss = mse(Z_dl(:,:,1:end-k+1), T);
%     loss = mse(Z_dl, T);
    
    % Calculate gradients wrt learnable parameters
    gradients = dlgradient(loss, net.Learnables);

end

function error = validate_net(net, X_test_queue, ns, k, mbs)
    % Reset necessary variables
    reset(X_test_queue);
    net = resetState(net);
    [X_test, T_test, ~] = next(X_test_queue);

    [Z_dl, ~] = minibatch_forecast(net, X_test, ns, k, mbs, false);

    rmses = zeros(1, mbs);
    for i = 1:mbs
        error = T_test(:,i,1:ns(i))-Z_dl(:,i,1:ns(i));
        error_squared = error.^2;
        rmses(i) = sqrt(mean(error_squared, 'all'));
    end

    error = sum(rmses.*ns(1:mbs))/sum(ns(1:mbs));
end

function [x, y, mask] = concatSequenceData(x, y)
[x, mask] = padsequences(x,2);
y = padsequences(y, 2);
% y = onehotencode(cat(2,y{:}),1);
end

