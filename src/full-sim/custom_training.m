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

mbqTrain = minibatchqueue(  dsTrain, 2, ...
                          'MiniBatchSize', 16, ...
                          'PartialMiniBatch', 'discard', ...
                          'MiniBatchFcn', @concatSequenceData, ...
                          'MiniBatchFormat', {'CTB', 'CTB'}, ...
                          'OutputAsDlarray', [1, 1]);

mbqVal = minibatchqueue(dsTest, 2, ...
                        "MiniBatchSize", 16, ...
                        "MiniBatchFcn", @concatSequenceData, ...
                        "MiniBatchFormat", {'CTB', 'CTB'}, ...
                        'OutputAsDlarray', [1, 1]);

disp("Minibatches created.")

%% Set up network
layers = setup_abl_residual_gru_no_output(17);
net = dlnetwork(layers);
executionEnvironment = "gpu";

k = 25;
validate = @(net) validate_net(net, mbqVal, val_ns, k);

numEpochs = 1;
miniBatchSize = 16; 
initialLearnRate = 0.001;
save_freq = 5; % epochs

figure
C = colororder;
lineLossTrain = animatedline(Color=C(2,:));
ylim([0 inf])
xlabel("Iteration")
ylabel("Loss")
grid on

disp("Network set up.")

%% Iterate
iteration = 0;
start = tic;
averageGrad = [];
averageSqGrad = [];

first_error = validate(net);

%%

for epoch = 1:numEpochs
    
    % Shuffle data
    shuffle(mbqTrain);

    % Loop over mini-batches
    while hasdata(mbqTrain)
%     for ct = 1:10
        iteration = iteration + 1

        % read mini-batch of data
        [X, Y] = next(mbqTrain);

        if canUseGPU
            X = gpuArray(X);
            Y = gpuArray(Y);
        end

        [loss,gradients,state] = dlfeval(@modelLoss,net, X,Y);
        net.State = state;

        [net,averageGrad,averageSqGrad] = adamupdate(net,gradients,averageGrad,averageSqGrad,iteration);
        
        D = duration(0,0,toc(start),'Format',"hh:mm:ss");
        loss = double(loss);
        addpoints(lineLossTrain,iteration,loss)
        title("Epoch: " + epoch + ", Elapsed: " + string(D))
        drawnow
    end

end

disp("Reached end of training.")

%% Functions
function [loss, gradients, state] = modelLoss(net, X, T)

    % Forward data through network
    [Z, state] = forward(net, X);
    
    % Calculate the loss
    loss = mse(Z, T);
    
    % Calculate gradients wrt learnable parameters
    gradients = dlgradient(loss, net.Learnables);

end

function [x, y] = concatSequenceData(x, y)
x = padsequences(x,2);  %, "Length", 1007); % for val set only
y = padsequences(y, 2); %, "Length", 1007); % for val set only
% y = onehotencode(cat(2,y{:}),1);
end

function error = validate_net(net, X_test_queue, ns, k)
    error = 0;
    % Reset necessary variables
    reset(X_test_queue);
    net = resetState(net);
    [X_test, T_test] = next(X_test_queue);

    % Make predictions up to n
    [Z, net_state] = predict(net, X_test);
    net.State = net_state;
    Z_og = extractdata(Z);

    % Setup forecast arrays
    last_preds = zeros(17, 16);
    all_preds = zeros(17, 16, k);

    % Get last prediction (at n)
    for i = 1:16
        last_preds(:,i,1) = Z_og(:,i,ns(i));
        if size(Z_og(:,i,:),3) > ns(i)
            Z_og(:,i,ns(i)+1:end) = zeros(17, 1, size(Z_og,3)-ns(i));
        end
    end

    all_preds(:,:,1) = last_preds;
    last_preds_dl = dlarray(last_preds, 'CBT');
    for i = 1:k-1
        [output, net_state] = predict(net, last_preds_dl);
        net.State = net_state;
        last_preds_dl = output;
        last_preds = extractdata(last_preds_dl);
        all_preds(:,:,i+1) = last_preds;
    end

    space_for_forecasts = zeros(17,16,k);
    cat(3, Z_og, space_for_forecasts);
    for i = 1:16
        Z_og(:,i,ns(i)+1:ns(i)+k) = all_preds(:,i,:);
    end

    Z_dl = dlarray(Z_og, 'CBT');

    rmses = zeros(1, 16);
    for i = 1:16
        error = T_test(:,i,1:ns(i))-Z_dl(:,i,1:ns(i));
        error_squared = error.^2;
        rmses(i) = sqrt(mean(error_squared, 'all'));
    end
%     parfor i = 1:numel(X_test)
%         pred = full_forecast_no_recur(net, X_test{i}, ns(i), k);
% %         size(pred)
%         pred = pred(:,end-k+1:end);
%         g_truth = X_test{i}(1:num_recur,ns(i)+1:ns(i)+k);
%         rmse = sqrt(immse(pred, single(g_truth)));
%         error = error + rmse;
%     end
%     error = error/numel(X_test);
end

