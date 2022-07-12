load('data/networks/toy-nets/SingleStepNet_071122.mat')
load('data/networks/toy-nets/SingleStepTestData_071122.mat')

X = XTest{6};
dt = X(1,2) - X(1,1);
T = TTest{6};
wp_vec = X(7:end,1); % (const) waypoint values

net = resetState(net);
offset = 200;
[net, Z] = predictAndUpdateState(net, X(:,1:offset));
    
numPredictionTimeSteps = 200;
t = X(1,offset);
Xt = [t; Z(:,end); wp_vec];
Y = zeros(6, numPredictionTimeSteps);

for n = 1:numPredictionTimeSteps
    [net, outputs] = predictAndUpdateState(net, Xt);
    t = t+dt;
    Y(:,n) = outputs;
    Xt = [t; outputs; wp_vec];
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
