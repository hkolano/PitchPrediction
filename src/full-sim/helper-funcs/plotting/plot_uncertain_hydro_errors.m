load("data/full-sim-with-hydro/rmses.mat")

zero_percent_avgs = [.0011; .0011; .0011; .0011; .0011]*.5;
one_percent_avgs = mean(one_percent_rmses, 2)*.5;
ten_percent_avgs = mean(ten_percent_rmses, 2)*.5;
fifty_percent_avgs = mean(fifty_percent_rmses, 2)*.5;

zero_percent_stdevs = zeros(5, 1);
one_percent_stdevs = std(one_percent_rmses, 1, 2);
ten_percent_stdevs = std(ten_percent_rmses, 1, 2);
fifty_percent_stdevs = std(fifty_percent_rmses, 1, 2);

mat = [zero_percent_avgs, one_percent_avgs, ten_percent_avgs, fifty_percent_avgs]';
stdevs = [zero_percent_stdevs, one_percent_stdevs, ten_percent_stdevs, fifty_percent_stdevs]';
param_names = categorical({'Manip. Added Mass', 'Manip. Drag', 'Vehicle Added Mass', 'Vehicle Lin. Drag', 'Vehicle Nonlin. Drag'});

[ngroups,nbars] = size(mat');
x = nan(nbars, ngroups);
close all

figure
b = bar(param_names, mat, 'FaceColor', 'flat');
hold on 

for i = 1:nbars
    x(i,:) = b(i).XEndPoints;
end

set(b(1), 'FaceColor', '#117733')
set(b(2), 'FaceColor', '#88CCEE')
set(b(3), 'FaceColor', '#DDCC77')
set(b(4), 'FaceColor', '#CC6677')
set(b, 'EdgeColor', [.3 .3 .3], ...
    'LineWidth', 1)

errorbar(x,mat,stdevs,'k','linestyle','none', 'Color', [.3 .3 .3], 'LineWidth', 1);
ylabel('Average RMSE')
legend("0 percent", "1 percent", "10 percent", "50 percent")
% title("RMSEs on Trajectories with Uncertain Hydrodynamics")

h = gca;
h.XAxis.TickLength = [0 0];
h.YAxis.TickLength = [0.02 0.02];

set(gca, ...
  'Box'         , 'off'     , ...
  'TickDir'     , 'out'     , ...
  'YMinorTick'  , 'on'      , ...
  'YGrid'       , 'on'      , ...
  'XGrid'       , 'off'     , ...
  'XColor'      , [.3 .3 .3], ...
  'YColor'      , [.3 .3 .3], ...
  'LineWidth'   , 1         );

set(gcf, ...
    'Position', [100 100 900 350]);

% model_series = [10 40 50 60; 20 50 60 70; 30 60 80 90]; 
% model_error = [1 4 8 6; 2 5 9 12; 3 6 10 13]; 
% b = bar(model_series, 'grouped');
% hold on
% % Calculate the number of groups and number of bars in each group
% [ngroups,nbars] = size(model_series);
% % Get the x coordinate of the bars
% x = nan(nbars, ngroups);
% for i = 1:nbars
%     x(i,:) = b(i).XEndPoints;
% end
% % Plot the errorbars
% errorbar(x',model_series,model_error,'k','linestyle','none');
% hold off
