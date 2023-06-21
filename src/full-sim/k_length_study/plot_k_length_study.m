%{
Plots the validation results from the simple stretch study and the
autoregressive study (Figure 9 in the submitted IROS paper). 
%}

sfs = 1:8;
lookahead_times = sfs*0.5;
auto_times = [.5 1 2 3 4];

load("data/networks/iros-nets/k_study_results2.mat")
stretch_forecast_avgs = mean(stretch_forecast_errors, 1);
stretch_forecast_avgs = stretch_forecast_avgs(sfs)

load("data/networks/iros-nets/auto_study_results.mat")

p = polyfit(lookahead_times, stretch_forecast_avgs, 2);
y_fit = polyval(p, [0.5:.1:4.25]);

p2 = polyfit(auto_times, auto_forecast_errors, 2);
y_fit2 = polyval(p2, [0.5:.1:4.25]);

close all

% f1 = figure('DefaultTextFontName', 'Helvetica');
f1 = figure
hold on
% line_thru = line(lookahead_times, rmse_avgs) %, '-d', 'Color', '#0072B2', 'LineWidth', 1.5)
fit_line = line([0.5:.1:4.25], y_fit);
fit_line2 = line([0.5:.1:4.25], y_fit2);
stretch_dots = plot(lookahead_times,stretch_forecast_avgs');
auto_dots = plot(auto_times, auto_forecast_errors);

my_xlab = xlabel('Prediction Length (s)');
my_ylab = ylabel('Validation RMSE');
xlim([0.25, 4.5])
ylim([-0.0, .25])

% dark green #117733
% light green #44AA99
% dark blue #332288
% light blue #88CCEE

% Fancying it up
% set(line_thru, ...
%     'Color', '#0072B2', ...
%     'LineWidth', 2);
set(fit_line, ...
    'Color', '#CC6677', ...
    'LineWidth', 2);

set(fit_line2, ...
    'Color', '#44AA99', ...
    'LineWidth', 2);

set(stretch_dots, ...
    'Marker', 's', ...
    'LineStyle', 'none', ...
    'Color', '#CC6677', ...
    'LineWidth', 2, ...
    'MarkerSize', 7, ...
    'MarkerEdgeColor', '#882255', ...
    'MarkerFaceColor', '#882255');

set(auto_dots, ...
    'LineStyle', 'none', ...
    'Marker', 's', ...
    'Color', '#88CCEE', ...
    'LineWidth', 2, ...
    'MarkerSize', 7, ...
    'MarkerEdgeColor', '#117733', ...
    'MarkerFaceColor', '#117733');

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
  'YTick'       , 0:.05:.25, ...
  'LineWidth'   , 1         );

set(gcf, ...
'Position', [100 100 700 250]);
leg = legend("Pitch-Only", "Autoregressive", 'Location', 'northwest');