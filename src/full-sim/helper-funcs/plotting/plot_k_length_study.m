% Plot results from k_length study

sfs = 1:6;

losses = [.0134, .0132, .0120; ...
    .0412, .0410, .0421; ...
    .1151, .1138, .1192; ...
    .2505, .2644, .2513; ...
    .5075, .4675, .4689; ...
    .8876, .8682, .8920]*10^(-3);

loss_avgs = mean(losses, 2)
loss_stdevs = std(losses, 0, 2);

% Last one is for sf = 6

RMSES = [.0051, .0051, .0049; ...
    .0091, .0090, .0091; ...
    .0151, .0150, .0154; ...
    .0223, .0227, .0223; ...
    .0316, .0304, .0303; ...
    .0418, .0414, .0419];

% last one is for sf = 6

rmse_avgs = mean(RMSES, 2)
rmse_stdevs = std(RMSES, 0, 2)

close all

figure
errorbar(sfs,rmse_avgs', rmse_stdevs','o','linestyle','none', 'LineWidth', 1.5);
xlabel('Stretch Factor')
ylabel('RMSE')
xlim([0.5, 6.5])
