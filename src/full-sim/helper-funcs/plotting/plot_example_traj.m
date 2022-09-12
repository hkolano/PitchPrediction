qs = readtable('data/traj_viz.csv');
rpys = readtable('data/traj_viz_rpy.csv');

times = [1:996]/50;

% dark blue: '#332288'
% dark green: '#117733'
% light green: '#44AA99'
% light blue: '#88CCEE'
% yellow: '#DDCC77'
% pink: '#CC6677'
% wine: '#882255'

snaps = [1 8 14 18 20];
fig_width = 600;
fig_height = 325;

%%
close all

figure
hold on
plot(times, qs.("qs4"), 'Color', '#117733', 'LineWidth', 2)
plot(times, qs.("qs5"), 'Color', '#88CCEE', 'LineWidth', 2)
plot(times, qs.("qs6"), "Color", '#CC6677', 'LineWidth', 2)
leg1 = legend("X position", "Y Position", "Z Position");
xlabel("Simulation Time (s)")
ylabel("Position (m)")

set(leg1, ...
    'Location', 'northwest');

set(gca, ...
  'Box'         , 'off'     , ...
  'TickDir'     , 'out'     , ...
  'TickLength'  , [.02 .02] , ...
  'XMinorTick'  , 'on'      , ...
  'YMinorTick'  , 'on'      , ...
  'YGrid'       , 'on'      , ...
  'XColor'      , [.3 .3 .3], ...
  'YColor'      , [.3 .3 .3], ...
  'LineWidth'   , 1         );

%%

snaps = [1 8 14 18 20];
fig_width = 600;
fig_height = 250;

% Joints Figure

close all
figure 
plot(times, qs.("qs7"), 'Color', '#117733', 'LineWidth', 2)
hold on
plot(times, qs.("qs8"), 'Color', '#88CCEE', 'LineWidth', 2)
plot(times, qs.("qs9"), 'Color', '#DDCC77', 'LineWidth', 2)
plot(times, qs.("qs10"), "Color", '#CC6677', 'LineWidth', 2)
xline(snaps, 'k', 'LineStyle', '-.', 'LineWidth', 1.5)

leg = legend("Joint 1", "Joint 2", "Joint 3", "Joint 4");
xlabel("Simulation Time (s)")
ylabel("Joint Angle (radians)")

set(leg, ...
'Location', 'northwest')

set(gcf, ...
    'Position', [100 100 fig_width fig_height])

set(gca, ...
  'Box'         , 'off'     , ...
  'TickDir'     , 'out'     , ...
  'TickLength'  , [.02 .02] , ...
  'XMinorTick'  , 'on'      , ...
  'YMinorTick'  , 'on'      , ...
  'YGrid'       , 'on'      , ...
  'XColor'      , [.3 .3 .3], ...
  'YColor'      , [.3 .3 .3], ...
  'LineWidth'   , 1         );

% RPY Figure

figure 
plot(times, rpys.("Roll"), 'Color', '#117733', 'LineWidth', 2)
hold on
yyaxis right
plot(times, rpys.("Pitch"), 'Color', '#88CCEE', 'LineWidth', 2)
yyaxis left
plot(times, rpys.("Yaw"), "Color", '#CC6677', 'LineWidth', 2)

xlabel("Simulation Time (s)")
ylabel("Roll and Yaw Angle (radians)")
xline(snaps, 'k', 'LineStyle', '-.', 'LineWidth', 1.5)
leg = legend("Roll", "Yaw", '', '', '', '', '', "Pitch");

set(leg, ...
'Location', 'northwest')

set(gcf, ...
    'Position', [100 fig_height+100 fig_width fig_height])

set(gca, ...
  'Box'         , 'off'     , ...
  'TickDir'     , 'out'     , ...
  'TickLength'  , [.02 .02] , ...
  'XMinorTick'  , 'on'      , ...
  'YMinorTick'  , 'on'      , ...
  'YGrid'       , 'on'      , ...
  'XColor'      , [.3 .3 .3], ...
  'YColor'      , [.3 .3 .3], ...
  'LineWidth'   , 1         );

yyaxis right
ylabel("Pitch Angle (radians)")
set(gca, ...
     'Box'         , 'off'     , ...
  'TickDir'     , 'out'     , ...
  'TickLength'  , [.02 .02] , ...
  'XMinorTick'  , 'on'      , ...
  'YMinorTick'  , 'on'      , ...
  'YGrid'       , 'on'      , ...
  'XColor'      , [.3 .3 .3], ...
  'YColor'      , '#099BE4', ...
  'LineWidth'   , 1         );