load('data/networks/one-d-nets/netv2.mat')
load('data/networks/one-d-nets/netv2testdata.mat')

X = XTest{2};
T = TTest{2};

net = resetState(net);
offset = 20;
[net, ~] = predictAndUpdateState(net, X(:,1:offset));

numTimeSteps = size(X, 2);
numPredictionTimeSteps = numTimeSteps - offset;
Y = zeros(1, numPredictionTimeSteps);

for t = 1:numPredictionTimeSteps
    Xt = X(:,offset+t);
    [net, Y(:,t)] = predictAndUpdateState(net,Xt);
end

figure
plot(T(:))
hold on
plot(offset+1:numTimeSteps, Y, '--')
xlabel("Time Step")
legend("Input", "Forecasted") 