function data = downselect_data_subset(idxs, full_data)
    for n = 1:numel(full_data)
        data{n} = full_data{n}(idxs, :); 
    end
end