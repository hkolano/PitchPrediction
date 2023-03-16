%{
Indicates which trajectories have normalized values of greater than 6
(trajectories that have values outside of 6 standard deviations from the
mean).

Used by import_full_data_meas_and_actual.m
%}
function bad_idxs = ID_outliers(sequence_data)

    max_vals = [];
%     disp(numel(sequence_data))
    for n = 1:numel(sequence_data)
        max_vals(n) = max(sequence_data{n}, [], 'all');
    end
    
%     hist(max_vals)

    bad_idxs = [];
    for idx = 1:length(max_vals)
        if max_vals(idx) > 6.0
            bad_idxs = [bad_idxs idx];
        end
    end
end