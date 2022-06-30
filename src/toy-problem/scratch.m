close all
load("data/networks/toy-nets/netv1testdata.mat")

v1s = XTest{1}(5,:);
v2s = XTest{1}(6,:);
v3s = XTest{1}(7,:);

v1s_mmean = movmean(v1s, 57);

figure
% subplot(3, 1, 1)
plot(v1s)
hold on
% plot(v1s_lpass)
plot(v1s_mmean)
xlabel("Time Step")
ylabel("Pitch Velocity")

% subplot(3, 1, 2)
% plot(v2s)
% xlabel("Time Step")
% ylabel("J1 Velocity")
% 
% subplot(3, 1, 3)
% plot(v3s)
% xlabel("Time Step")
% ylabel("J2 Velocity")
