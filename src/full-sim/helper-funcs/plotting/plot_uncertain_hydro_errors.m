one_percent_rmses = [.0041, .0026, .0037; ...
    .0056, .0052, .0045; ...
    .0060, .0101, .0087; ...
    .0075, .0065, .0044; ...
    .0039,.0047, .0094];

ten_percent_rmses = [.0030, .0104, .0056; ...
    .0049, .0080, .0074; ...
    .0069, .0038, .0041; ...
    .0057, .0054, .0080; ...
    .0070, .0077, .0042];

fifty_percent_rmses = [.0040, .0025, .0071; ...
    .0053, .0078, .0039; ...
    .0095, .0259, .0133; ...
    .0124, .0108, .0182; ...
    .0115, .0063, .0021];

zero_percent_avgs = [.0014; .0014; .0014; .0014; .0014];
one_percent_avgs = mean(one_percent_rmses, 2);
ten_percent_avgs = mean(ten_percent_rmses, 2);
fifty_percent_avgs = mean(fifty_percent_rmses, 2);

zero_percent_stdevs = zeros(5, 1);
one_percent_stdevs = std(one_percent_rmses, 1, 2);
ten_percent_stdevs = std(ten_percent_rmses, 1, 2);
fifty_percent_stdevs = std(fifty_percent_rmses, 1, 2);

mat = [zero_percent_avgs, one_percent_avgs, ten_percent_avgs, fifty_percent_avgs]';
stdevs = [zero_percent_stdevs, one_percent_stdevs, ten_percent_stdevs, fifty_percent_stdevs]';
param_names = categorical({'Manip. a.m.', 'Manip. drag', 'Vehicle a.m.', 'Vehicle l. drag', 'Vehicle non. drag'});

[ngroups,nbars] = size(mat');
x = nan(nbars, ngroups);
close all

figure
b = bar(param_names, mat);
hold on 

for i = 1:nbars
    x(i,:) = b(i).XEndPoints
end

errorbar(x,mat,stdevs,'k','linestyle','none');
ylabel('Average RMSE')
legend("0 percent", "1 percent", "10 percent", "50 Percent")

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
