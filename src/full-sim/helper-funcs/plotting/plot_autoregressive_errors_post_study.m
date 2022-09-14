folder = "data/networks/full-nets/10Hz_alltrajs_k5";
take_n = 3;

if take_n == 1
    epochs = [5:5:50];
    color = 'b';
else
    epochs = [5:5:45, 46:1:50];
    color = 'c';
end

% figure

for epoch_n = 1:length(epochs)
    ep = epochs(epoch_n);
    load(fullfile(folder, strcat("take", string(take_n), "_", string(ep), "epochs.mat")))

    hold on
    grid on
    plot(start_it:1:end_it, movmean(these_epochs_training_RMSE_vec, 20), color)
end