function replot_training_plot(info)
figure
plot(info.ValidationLoss(info.ValidationLoss>0))
hold on
plot(info.TrainingLoss(info.ValidationLoss>0))
legend('Validation', 'Training')
xlabel('Validation #')
ylabel("Loss")
end