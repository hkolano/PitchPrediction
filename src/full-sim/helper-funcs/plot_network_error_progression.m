function plot_network_error_progression()

% files = dir(fullfile(folder_path, '*.mat'))
% save_pt_its = [250, 500, 750, 1000, 1250, 1500, 1750, 2000, 2500, 3000, 3500, 4000];
fds = fileDatastore("data/networks/full-nets/increasing_k_nets", 'ReadFcn', @importdata);
fullFileNames = fds.Files;
numFiles = length(fullFileNames);

figure;
hold on

for m = 1:numFiles
%     file_name = fullfile("data/networks/full-nets/retrained_nets", strcat("net_A_1_", string(save_pt_its(m)), "its.mat"))
%     baseFileName = files(m).name
%     fullFileName = fullfile(files, baseFileName)
    load(fullFileNames{m});

    its = start_it-.8:0.2:end_it;

    plot(its, movmean(training_rmse_vec, 21), 'b')
    hold on
    plot(start_it+24:25:end_it, error_vec, 'r')

end

xlabel("Iteration Number")
ylabel("RMSE")
title("Noodle")
grid on
legend("Training RMSE", "Validation RMSE")

drawnow;

end