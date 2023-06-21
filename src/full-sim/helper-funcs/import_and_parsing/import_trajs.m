function data = import_trajs(folder)
% Import the data from 
    fds = fileDatastore(folder,"ReadFcn",@read_to_array);
    data = readall(fds);
    
    % Preview the data
%     figure
%     tiledlayout(7,3)
%     for m = 1:21
%         nexttile
%         stackedplot(data{m})
%     
%         xlabel("Time Step")
%     end

    function data = read_to_array(input)
        data = readtable(input);
        data = table2array(data)';
    end


end

