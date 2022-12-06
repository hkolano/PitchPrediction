load('data/channel_dict.mat');
val_set = 'data/full-data-matlab/val_set_final_50hz.mat';
load('data/full-data-matlab/FullData_17chan_10Hz.mat');

%%
load(val_set);
XTest_subset = XTest_10hz(val_idxs);
val_ns = floor(val_ns./5);

ks = [5 10 20 30 40];

auto_forecast_errors = zeros(1, length(ks));

for k_idx = 1:length(ks)
    k = ks(k_idx)
    load(strcat("data\networks\full-nets\10Hz_alltrajs_k", string(k), "\take3_50epochs.mat"))
    error = validate_pitch_on_forecast_only(net, XTest_subset, val_ns, k, p, pitch_idx)
    auto_forecast_errors(k_idx) = error;
end