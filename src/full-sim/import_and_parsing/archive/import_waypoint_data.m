function data = import_waypoint_data(file)
% Import the data from 
    ds = datastore(file);
    data = readall(ds);
    data = table2array(data)';
    
%     function data = read_to_array(input)
%         data = readtable(input);
%         data = table2array(data)';
%     end

end

