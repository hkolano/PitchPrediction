%% Load data
% load('data/full-data-matlab/channel_subgroups/no_goal_poses/no_manip_vels/no_goal_vels/data_without_xyz_poses.mat')
folder_trajs = "data\full-sim-data\data-no-orientation";
folder_rpys = "data\full-sim-data\data-rpy";

rpy_tables_ds = fileDatastore(folder_rpys, "ReadFcn", @read_rpy_to_table);

foo = @(input) read_trajs_and_combine(input, rpy_tables_ds);
traj_ds = fileDatastore(folder_trajs, "ReadFcn", foo);
% data = readall(traj_ds);

%% Set up partitions
nFiles = length(traj_ds.Files);
n95percent = round(0.95*nFiles);

traj_ds_Train = subset(traj_ds, 1:n95percent);
traj_ds_Val = subset(traj_ds, n95percent+1:nFiles);
traj_data_Train = readall(traj_ds_Train);
traj_data_Val = readall(traj_ds_Val);

%% Set up network
layers = setup_abl_residual_gru(17);

numEpochs = 10;
miniBatchSize = 16; 
initialLearnRate = 0.001; 

mbq = minibatchqueue(traj_ds_Train, ...
    MiniBatchSize = miniBatchSize);

%% Functions
function data = read_trajs_and_combine(input, rpy_table)
    data_traj = readtable(input);
    data_rpy = read(rpy_table);
    combo_table = [data_traj data_rpy];
    combo_table = removevars(combo_table, {'qs4', 'qs5', 'qs6', 'vs7', 'vs8', 'vs9', 'vs10'});
    data = table2array(combo_table);
end

function data = read_rpy_to_table(input)
    data = readtable(input);
%     data = table2array(data)';
end

% function data = import_traj_no_orientation(folder)
% % Import the data from 
%     fds = fileDatastore(folder,"ReadFcn",@read_to_array);
%     data = readall(fds);
%     function data = smooth_velocities(data)
%         window = 15;
%         for data_idx = 8:17
%             for i = 1:numel(data)
%                 data{i}(data_idx,:) = movmean(data{i}(data_idx,:), window);
%             end
%         end
%     end
% 
% end



%%
% numChannels = size(XTrain{1}, 1);
% numRecChannels = size(TTrain{1}, 1);
% layers = setup_full_residual_gru(numChannels, numRecChannels);
% 

% 
% mbq = minibatchqueue(XTrain, ...
%     MiniBatchSize = miniBatchSize);
% 
% %%
% %     MaxEpochs=5, ...
% init_options = trainingOptions("adam", ...
%     MaxEpochs=100, ...
%     MiniBatchSize=16, ...
%     InitialLearnRate=0.001, ...
%     LearnRateDropPeriod = 5,...
%     LearnRateSchedule= 'piecewise', ...
%     LearnRateDropFactor= .9, ...
%     SequencePaddingDirection="right", ...
%     Plots="training-progress", ...
%     Shuffle='every-epoch', ...
%     ValidationData={XTest, TTest}, ...
%     ValidationFrequency = 60, ...
%     OutputNetwork='best-validation-loss');
% [net, info] = trainNetwork(XTrain,TTrain,layers,init_options);
% %     
% outputFile = fullfile("data/networks/full-nets", 'SingleStepNet_18chan_384units.mat');
% save(outputFile, 'net', 'info');