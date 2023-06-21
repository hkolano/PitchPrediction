function bad_idxs = ID_outliers(sequence_data)

    max_vals = [];
    for n = 1:numel(sequence_data)
        max_vals(n) = max(sequence_data{n}, [], 'all');
    end
    
%     hist(max_vals)

    bad_idxs = [];
    for idx = 1:length(max_vals)
        if max_vals(idx) > 5.0
            bad_idxs = [bad_idxs idx];
        end
    end
end