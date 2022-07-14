X = XTest{5};
n = 200;
k = 200;
make_plot = true;

net = resetState(net);

% Extract the constant values (waypoints, dt)
const_vec = X(7:end,1);

% Predict and update until the forecasting start
[net, Z] = predictAndUpdateState(net, X(:,1:n));

% Get the last output
Xt = [Z(:,end); const_vec];
% Setup forecast array
Y = zeros(6, k);
%     Y(:,1) = Z(:,end);

% Generate the forecast
for i = 1:k
    [net, outputs] = predictAndUpdateState(net, Xt);
    Y(:,i) = outputs;
    Xt = [outputs; const_vec];
end