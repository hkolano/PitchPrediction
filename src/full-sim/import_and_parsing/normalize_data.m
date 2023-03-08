%{
Finds the mean and standard deviation of each data stream; saves those to
"params". "data" is the trajectory data normalized to have a mean of 0 and
a standard deviation of 1. 

Used by import_full_data_meas_and_actual.m
%}
function [data, params] = normalize_data(data)
    mu = mean([data{:}],2);
    sig = std([data{:}],0,2);
    
    for i = 1:numel(data)
        data{i} = (data{i} - mu) ./ sig;
    end
    
    params.mu = mu;
    params.sig = sig;
end