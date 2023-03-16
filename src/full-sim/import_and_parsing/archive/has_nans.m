load('data/full-data-matlab/FullData_081022.mat')  

all_nans = 0;
% nan_array = [];

for i = 1:numel(XTest)
    num_nans = sum(isnan(XTest{i}), 'all');
%     nan_array(i) = num_nans;
    if num_nans ~= 0
        all_nans = all_nans + 1
    end
end

disp(nan_array)