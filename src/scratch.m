load('data/full-data-matlab/FullData_17chan_10Hz.mat')
load('C:\Users\OSU\Documents\GitHub\PitchPrediction\data\networks\full-nets\10Hz_alltrajs_k5\take1_5epochs.mat')

plot_autoreg_forecast(net, XTest_10hz{1}, 50, 5, p, 16, 100)

load('C:\Users\OSU\Documents\GitHub\PitchPrediction\data\networks\full-nets\10Hz_alltrajs_k5\take1_25epochs.mat')

plot_autoreg_forecast(net, XTest_10hz{1}, 50, 5, p, 16, 100)

load('C:\Users\OSU\Documents\GitHub\PitchPrediction\data\networks\full-nets\10Hz_alltrajs_k5\take1_50epochs.mat')

plot_autoreg_forecast(net, XTest_10hz{1}, 50, 5, p, 16, 100)

