function plot_training_errors(train_errors, validation_errors, plot_title)
    figure
    plot(movmean(validation_errors, 21), 'b')
    hold on
    plot((0:length(train_errors)-1)*100, train_errors)
    
    xlabel("Iteration Number")
    ylabel("RMSE")
    title(plot_title)
    grid on
    legend("Training RMSE", "Validation RMSE")
end