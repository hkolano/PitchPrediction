load('data/networks/one-d-nets/netv2.mat')
load('data/networks/one-d-nets/netv2testdata.mat')

X = XTest{3};
dt = X(1,2) - X(1,1);
T = TTest{3};

net = resetState(net);
offset = 100;
[net, Z] = predictAndUpdateState(net, X(:,1:offset));
    
numPredictionTimeSteps = 100;
t = X(1,offset);
Xt = [t; Z(:,end)];
Y = zeros(1, numPredictionTimeSteps);

for n = 1:numPredictionTimeSteps
    [net, Y(:,n)] = predictAndUpdateState(net, Xt);
    t = t+dt;
    Xt = [t; Y(:,n)];
end

timesteps = offset+numPredictionTimeSteps;

%% Plotting
figure
plot(T(:), 'g')
hold on
% plot(T(offset+1:timesteps), 'c')
plot(offset+1:timesteps, Y, 'm--')
xlabel("Time Step")
xline(offset, 'k-.')
legend("Ground Truth", "Forecasted", "Prediction Start")
