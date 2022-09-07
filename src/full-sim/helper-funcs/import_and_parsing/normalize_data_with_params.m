function data = normalize_data_with_params(data, params)   
    for i = 1:numel(data)
        data{i} = (data{i} - params.mu) ./ params.sig;
    end
end