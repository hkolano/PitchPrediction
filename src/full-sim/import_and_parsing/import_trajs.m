%{
Imports all the csv files from a folder into one matlab table

Used by import_full_data_meas_and_actual.m
%}
function data = import_trajs(folder)
% Import the data from 
    fds = fileDatastore(folder,"ReadFcn",@read_to_array);
    data = readall(fds);

    function data = read_to_array(input)
        data = readtable(input);
        data = table2array(data)';
    end
end
