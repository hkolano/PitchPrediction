% Plot results from k_length study

sfs = 1:9;
lookahead_times = sfs*0.5;

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
    .0418, .0414, .0419; ...
    .0531, .0534, .0534; ...
    .0713, .0695, .0710; ...
    .0895, .0904, .0913];

% last one is for sf = 6

rmse_avgs = mean(RMSES, 2)
rmse_stdevs = std(RMSES, 0, 2)

p = polyfit(lookahead_times, rmse_avgs, 2);
y_fit = polyval(p, [0.5:.1:4.75]);

close all

% f1 = figure('DefaultTextFontName', 'Helvetica');
f1 = figure
hold on
% line_thru = line(lookahead_times, rmse_avgs) %, '-d', 'Color', '#0072B2', 'LineWidth', 1.5)
fit_line = line([0.5:.1:4.75], y_fit);
er_bars = plot(lookahead_times,rmse_avgs');
my_xlab = xlabel('Prediction Length (s)');
my_ylab = ylabel('RMSE');
my_title = title('RMSEs on Increasing Prediction Length');
xlim([0.25, 5])
ylim([0, .1])

% Fancying it up
% set(line_thru, ...
%     'Color', '#0072B2', ...
%     'LineWidth', 2);
set(fit_line, ...
    'Color', '#CC6677', ...
    'LineWidth', 2);
set(er_bars, ...
    'LineStyle', 'none', ...
    'Marker', 's', ...
    'Color', [.3 .3 .3], ...
    'LineWidth', 1, ...
    'MarkerSize', 7, ...
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
  'YTick'       , 0:.02:.1, ...
  'LineWidth'   , 1         );

set(gcf, ...
'Position', [100 100 550 300]);