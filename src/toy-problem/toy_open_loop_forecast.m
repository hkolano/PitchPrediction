load('data/networks/toy-nets/netv1.mat')
load('data/networks/toy-nets/netv1testdata.mat')

X = XTest{1};
T = TTest{1};

net = resetState(net);
offset = 50;
[net, ~] = predictAndUpdateState(net, X(:,1:offset));

numTimeSteps = size(X, 2);
numPredictionTimeSteps = numTimeSteps - offset;
Y = zeros(6, numPredictionTimeSteps);
 
for t = 1:numPredictionTimeSteps
    Xt = X(:,offset+t);
    [net, Y(:,t)] = predictAndUpdateState(net,Xt);
end
%% Plotting
figure
plot(T(1,:))
hold on
plot(offset+1:numTimeSteps, Y(1,:), '--')
xlabel("Time Step")
legend("Input", "Forecasted") 