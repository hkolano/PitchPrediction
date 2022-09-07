num_inputs = [33, 29, 21, 18, 14, 11, 9, 5, 3];
num_inputs = num_inputs-1;

load("data\networks\full-nets\ablation_rmses.mat");

all_losses = [.000115, .0001291, .000094; ...
    .00011085, .00010855, .00009955; ...
    .0001077, .0001084, .0001034; ...
    .0000944, .0001038, .0000890; ...
    .0001022, .0000988, .0000902; ...
    .0000900, .0001128, .0001100; ...
    .0001211, .0001018, .0001162; ...
    .0001408, .0001588, .0001347; ...
    .0001765, .0001742, .0001853];

loss_avgs = mean(all_losses, 2);
loss_stdevs = std(all_losses, 0, 2);

rmse_avgs = mean(ablation_rmses, 2);
rmse_stdevs = std(ablation_rmses, 0, 2);
    
figure
% scatter(num_inputs, loss_avgs, 'MarkerEdgeColor', '#56B4E9', 'LineWidth', 2)
hold on
er_bars = errorbar(num_inputs, rmse_avgs, rmse_stdevs); %'Color', '#56B4E9')
my_xlab = xlabel("Number of Channel Inputs");
my_ylab = ylabel("Prediction RMSE");
set(gca, 'XDir', 'reverse')
ylim([.012, .02])
my_title = title('Network Performance on Decreasing Input Channels');

% set(fit_line, ...
%     'Color', '#CC6677', ...
%     'LineWidth', 2);
set(er_bars, ...
    'LineStyle', 'none', ...
    'Marker', 's', ...
    'Color', [.3 .3 .3], ...
    'LineWidth', 1.25, ...
    'MarkerSize', 7.5, ...
    'MarkerEdgeColor', '#882255', ...
    'MarkerFaceColor', '#882255');


% set( gca                       , ...
%     'FontName'   , 'Helvetica' );
% set([my_title, my_xlab, my_ylab], ...
%     'FontName'   , 'AvantGarde');
set( my_title                    , ...
    'FontSize'   , 12          , ...
    'FontWeight' , 'bold'      );
set([my_xlab, my_ylab]  , ...
    'FontSize'   , 10          );
set(gca, ...
  'Box'         , 'off'     , ...
  'TickDir'     , 'out'     , ...
  'TickLength'  , [.02 .02] , ...
  'XMinorTick'  , 'on'      , ...
  'YMinorTick'  , 'on'      , ...
  'YGrid'       , 'on'      , ...
  'XGrid'       , 'off'     , ...
  'XColor'      , [.3 .3 .3], ...
  'YColor'      , [.3 .3 .3], ...
  'LineWidth'   , 1         );
