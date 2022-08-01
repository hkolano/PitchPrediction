load('data/networks/toy-nets/SingleStepNet_071222_v1.mat')
load('data/networks/toy-nets/SingleStepTestData_071122.mat')

idx = 2;

X = XTest{idx};
T = TTest{idx};

net = resetState(net);
offset = 5;
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
t = tiledlayout(3,2, 'TileSpacing', 'Compact');
ylabels = ["Vehicle Pitch", "Joint 1", "Joint 2", "", "", ""];
order = [1 4 2 5 3 6];
titles = ["Joint Position (radians)", "Joint Velocity (rad/s)", "", "", "", ""];

for i = 1:6
    idx = order(i);
    ax = nexttile;
    plot((1:numTimeSteps)/50, T(idx,:).*p.sig(idx)+p.mu(idx), 'LineWidth', 2, 'Color', '#FFC20A')
    hold on
    plot((offset+1:numTimeSteps)/50, Y(idx,:).*p.sig(idx)+p.mu(idx), '--', 'LineWidth', 2, 'Color', '#0C7BDC')
    ylabel(ylabels(idx))
    title(titles(i))
    legend("Ground Truth", "Forecasted") 
    
end
title(t, "One Step Ahead Predictions on Toy Problem")
xlabel(t, "Time (s)")

% nexttile(1).ylabel("Vehicle Pitch (radians)")

