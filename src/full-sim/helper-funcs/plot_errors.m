function plot_errors(train_errors, validation_errors, val_freq, plot_title)
    close all

    figure
    plot(movmean(train_errors, 5), 'b')
    hold on
    plot((0:length(validation_errors)-1)*val_freq, validation_errors)
    
    xlabel("Iteration Number")
    ylabel("RMSE")
    title(plot_title)
    grid on
    legend("Training RMSE", "Validation RMSE")

    drawnow;
end