function data = import_toy_waypoint_data()
% Import the data from 
    ds = datastore("data/toy-data-waypoints.csv");
    data = readall(ds);
    data = table2array(data)';
    
%     function data = read_to_array(input)
%         data = readtable(input);
%         data = table2array(data)';
%     end

end

