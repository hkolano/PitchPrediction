function data = import_traj_no_orientation(folder)
% Import the data from 
    fds = fileDatastore(folder,"ReadFcn",@read_to_array);
    data = readall(fds);
    
%     figure 
%     plot(data{2}(5,:))
%     hold on
%     disp("Smoothing the velocities")
    data = smooth_velocities(data);
%     plot(data{2}(5,:))

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

    function data = smooth_velocities(data)
        window = 15;
        for data_idx = 8:17
            for i = 1:numel(data)
                data{i}(data_idx,:) = movmean(data{i}(data_idx,:), window);
            end
        end
    end

end

