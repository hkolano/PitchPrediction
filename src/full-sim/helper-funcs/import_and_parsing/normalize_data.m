function [data, params] = normalize_data(data)
    mu = mean([data{:}],2);
    sig = std([data{:}],0,2);
    
    for i = 1:numel(data)
        data{i} = (data{i} - mu) ./ sig;
    end
    
    params.mu = mu;
    params.sig = sig;
end