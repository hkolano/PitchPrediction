load('data/networks/toy-nets/netv1.mat')
load('data/networks/toy-nets/netv1testdata.mat')

X = XTest{6};
dt = X(1,2) - X(1,1);
T = TTest{6};

net = resetState(net);
offset = 1000;
[net, Z] = predictAndUpdateState(net, X(:,1:offset));
    
numPredictionTimeSteps = 2000;
t = X(1,offset);
Xt = [t; Z(:,end)];
Y = zeros(6, numPredictionTimeSteps);

for n = 1:numPredictionTimeSteps
    [net, outputs] = predictAndUpdateState(net, Xt);
    t = t+dt;
    Y(:,n) = outputs;
    Xt = [t; outputs];
end

timesteps = offset+numPredictionTimeSteps;

%% Plotting
figure
plot(T(1,:), 'g')
hold on
% plot(T(offset+1:timesteps), 'c')
plot(offset+1:timesteps, Y(1,:), 'm--')
xlabel("Time Step")
xline(offset, 'k-.')
legend("Ground Truth", "Forecasted", "Prediction Start")
