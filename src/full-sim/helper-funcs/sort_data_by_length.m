%{
Takes XTrain or XTest and sorts them by length in descending order.

Last modified 12/6/22
%}
function sorted_data = sort_data_by_length(data)
    [~,I] = sort(cellfun(@length, data));
    sorted_data = data(flip(I, 2));
end