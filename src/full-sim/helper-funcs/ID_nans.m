function bad_idxs = ID_nans(data)

bad_idxs = [];

for i = 1:numel(data)
    num_nans = sum(isnan(data{i}), 'all');
    if num_nans ~= 0
        bad_idxs = [bad_idxs, i];
    end
end
end