function data = import_orientations(folder)
% Import the data from 
    disp("Starting function")
    fds = fileDatastore(folder,"ReadFcn",@read_to_array);
    disp("filestore defined")
    data = readall(fds);

    function data = read_to_array(input)
        data = readtable(input);
        data = table2array(data)';
    end

end

