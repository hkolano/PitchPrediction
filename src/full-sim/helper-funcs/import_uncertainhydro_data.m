% function train_toy_data()
%% Data import and processing
path = "data/full-sim-with-hydro/";
param_names = ["arm-added-mass", "arm-linear-drag", "vehicle-added-mass", "vehicle-linear-drag", "vehicle-quadratic-drag"];

for param_id = 1:length(param_names)
    param = param_names(param_id);
    disp(strcat("Dealing with param: ", string(param)))
    for model_num = 1:3
        disp(strcat("Parsing model:", string(model_num)))
        sequence_data = import_traj_no_orientation(strcat(path, "single-model-50percent/", param, "/data-no-orientation-model", string(model_num)));
        orientation_data = import_orientations(strcat(path, "single-model-50percent/", param, "/data-rpy-model", string(model_num)));
        
        [p, XData] = create_XData(sequence_data, orientation_data); 
        
        % Save the output
        outputFile = fullfile(strcat(path, "single-model-50percent/", param), strcat('model', string(model_num), '_data.mat'));
        save(outputFile, 'XData', 'p')
    end
end

function [p, XData] = create_XData(sequence_data, orientation_data)

    for idx = 1:numel(sequence_data)
        seq_data{idx} = [sequence_data{idx}; orientation_data{idx}];
    end
    
    %%
    nan_IDs = ID_nans(seq_data)
    for idx = length(nan_IDs):-1:1
        seq_data(nan_IDs(idx)) = [];
    end
    
    [seq_data, p] = normalize_data(seq_data);
    bad_ids = ID_outliers(seq_data)
    
    %%
    % Remove outliers (from highest idx to lowest idx)
    for idx = length(bad_ids):-1:1
        seq_data(bad_ids(idx)) = [];
    end
    
    numChannels = size(seq_data{1}, 1);
    
    for n = 1:numel(seq_data)
        X = seq_data{n};
        XData{n} = [X(:,1:end-1)];
    end
    
    %% Sort by sequence length
    for i=1:numel(XData)
        sequence = XData{i};
        sequenceLengths(i) = size(sequence,2);
    end
    
    [sequenceLengths,idx] = sort(sequenceLengths,'descend');
    XData = XData(idx);
end




