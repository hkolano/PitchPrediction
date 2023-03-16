%{
Need to manually input the RMSE values gained from ablation_study_recursive
and ablation_study_benchmark. Plots the validation RMSEs vs how many input
channels were used. 

Last modified 3/8/23
%}

new_rmses = [.0203, .0177, .0163, .0151, .0144, .0142, .0135, .0136, .0140];
new_rmse_stdevs = [0, 0., 0, 0, 0, 0, 0, 0, 0];
num_inputs = [40, 37, 34, 26, 22, 14, 12, 8, 6];
    
figure
% scatter(num_inputs, loss_avgs, 'MarkerEdgeColor', '#56B4E9', 'LineWidth', 2)
hold on
er_bars = errorbar(num_inputs, new_rmses, new_rmse_stdevs); %'Color', '#56B4E9')
my_xlab = xlabel("Number of Channel Inputs");
my_ylab = ylabel("Prediction RMSE");
set(gca, 'XDir', 'reverse')
ylim([.013, .0195])
xlim([0, 42])
% my_title = title('Network Performance on Decreasing Input Channels');

% set(fit_line, ...
%     'Color', '#CC6677', ...
%     'LineWidth', 2);
set(er_bars, ...
    'LineStyle', 'none', ...
    'Marker', 's', ...
    'Color', [.3 .3 .3], ...
    'LineWidth', 1.0, ...
    'MarkerSize', 7.5, ...
    'MarkerEdgeColor', '#882255', ...
    'MarkerFaceColor', '#882255');


% set( gca                       , ...
%     'FontName'   , 'Helvetica' );
% set([my_title, my_xlab, my_ylab], ...
%     'FontName'   , 'AvantGarde');
% set( my_title                    , ...
%     'FontSize'   , 12          , ...
%     'FontWeight' , 'bold'      );
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

set(gcf, ...
    'Position', [100 100 600 250]);
