function data = import_toy_training_data(folder)
% Import the data from 
    fds = fileDatastore(folder,"ReadFcn",@read_to_array);
    data = readall(fds);
    
%     figure 
%     plot(data{2}(5,:))
%     hold on
%     data = smooth_velocities(data);
%     plot(data{2}(5,:))

    % Preview the data
    % figure
    % tiledlayout(2,2)
    % for i = 1:4
    %     nexttile
    %     stackedplot(data{i})
    % 
    %     xlabel("Time Step")
    % end

    function data = read_to_array(input)
        data = readtable(input);
        data = table2array(data)';
    end

    function data = smooth_velocities(data)
        window = 21;
        for i = 1:numel(data)
            data{i}(5,:) = movmean(data{i}(5,:), window);
            data{i}(6,:) = movmean(data{i}(6,:), window);
            data{i}(7,:) = movmean(data{i}(7,:), window);
        end
    end

end

