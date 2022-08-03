function plot_training_errors(train_errors, validation_errors, plot_title)
    figure
    plot(movmean(train_errors, 51), 'b')
    hold on
    plot((0:length(validation_errors)-1)*250, validation_errors)
    
    xlabel("Iteration Number")
    ylabel("RMSE")
    title(plot_title)
    grid on
    legend("Training RMSE", "Validation RMSE")
end