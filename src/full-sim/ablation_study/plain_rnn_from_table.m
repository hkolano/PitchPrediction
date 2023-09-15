%{
Trains a neural network on training data from Julia, all saved as tables. 

Assumes that data is input at 25 Hz.

Make sure data input is the correct one, and that MaxEpochs is set to 50 in
define_new_opts. 

Last modified 9/12/23
%}
% Load in the data
load("data/full-sim-data-091023/NormalizedTableTrainData.mat")
train_data_table = sequence_data_table;
load("data/full-sim-data-091023/NormalizedTableValData.mat")
val_data_table = sequence_data_table;
load("data/full-sim-data-091023/NormalizedTableTestData.mat")
test_data_table = sequence_data_table;

% define constants from the data
des_pred_length = 2.; % seconds
% sim_dt = train_data_table{1}{2,"time_secs"} - train_data_table{1}{1, "time_secs"};
sim_dt = .04;
des_pred_steps = round(des_pred_length/sim_dt);

% which columns are we attempting to predict?
gt_pitch_column_name = "qs2";
gt_roll_column_name = "qs1";
input_data_columns = ["qs1", "qs2", "qs7", "qs8", "qs9", "qs10", "vs1", "vs2", "vs3", "vs7", "vs8", "vs9", "vs10"];

% empty cells for populating formatted inputs & outputs
gt_data = {};
train_data_array = {};
gt_array = {};

val_outputs_table = {};
val_data_array = {};
val_outputs_array = {};

test_outputs_table = {};
test_data_array = {};
test_outputs_array = {};

disp('Data imported.')

%%
% ------------------------------------------------------------------------
%                      Format data for the network
% ------------------------------------------------------------------------
% Format training data
for n = 1:numel(train_data_table)
    train_data_table{n} = train_data_table{n}(:,input_data_columns);
    gt_data{n} = array2table(zeros(height(train_data_table{n})-des_pred_steps, des_pred_steps*2));
    sequence_length = height(train_data_table{n})-des_pred_steps;
    for i = 1:sequence_length
        gt_data{n}{i,1:des_pred_steps} = train_data_table{n}{i+1:i+des_pred_steps, gt_roll_column_name}';
        gt_data{n}{i,des_pred_steps+1:end} = train_data_table{n}{i+1:i+des_pred_steps, gt_pitch_column_name}';
    end
    train_data_array{n} = table2array(train_data_table{n}(1:sequence_length,:))';
    gt_array{n} = table2array(gt_data{n})';
end

% Format validation data
for n = 1:numel(val_data_table)
    val_data_table{n} = val_data_table{n}(:,input_data_columns);
    val_outputs_table{n} = array2table(zeros(height(val_data_table{n})-des_pred_steps, des_pred_steps*2));
    sequence_length = height(val_data_table{n})-des_pred_steps;
    for i = 1:sequence_length
        val_outputs_table{n}{i,1:des_pred_steps} = val_data_table{n}{i+1:i+des_pred_steps, gt_roll_column_name}';
        val_outputs_table{n}{i,des_pred_steps+1:end} = val_data_table{n}{i+1:i+des_pred_steps, gt_pitch_column_name}';
    end
    val_data_array{n} = table2array(val_data_table{n}(1:sequence_length,:))';
    val_outputs_array{n} = table2array(val_outputs_table{n})';
end

% Format test data
for n = 1:numel(test_data_table)
    test_data_table{n} = test_data_table{n}(:, ["roll", "pitch", "axis_e_pos", "axis_d_pos", "axis_c_pos", "axis_b_pos", "x_angvel", "y_angvel", "z_angvel", "axis_e_vel", "axis_d_vel", "axis_c_vel", "axis_b_vel"]);
    test_outputs_table{n} = array2table(zeros(height(test_data_table{n})-des_pred_steps, des_pred_steps*2));
    sequence_length = height(test_data_table{n})-des_pred_steps;
    for i = 1:sequence_length
        test_outputs_table{n}{i,1:des_pred_steps} = test_data_table{n}{i+1:i+des_pred_steps, "roll"}';
        test_outputs_table{n}{i,des_pred_steps+1:end} = test_data_table{n}{i+1:i+des_pred_steps, "pitch"}';
    end
    test_data_array{n} = table2array(test_data_table{n}(1:sequence_length,:))';
    test_outputs_array{n} = table2array(test_outputs_table{n})';
end

disp('Data formatted.')

%%
% ------------------------------------------------------------------------
%                            Train Network
% ------------------------------------------------------------------------
numUnits = 384;

% How many feature channels remain?
numChannels = width(train_data_table{1});

% Initialize the neural network
layers = setup_lookahead_rnn(numChannels, des_pred_steps*2, numUnits);
init_options = define_new_opts(val_data_array', val_outputs_array');

disp('Starting to train network.')

% for take_n = 1:3
    [net, info] = trainNetwork(train_data_array',gt_array',layers,init_options);
    
    loss = info.FinalValidationLoss;
    RMSEs = info.FinalValidationRMSE;
    %     
    outputFile = strcat("data/networks/posthinsdalenets/rnn.mat");
    save(outputFile, 'net', 'info');
% end

% To close all plots:
% delete(findall(0));

disp('Network trained and saved.')

%%
% ------------------------------------------------------------------------
%                      Testing on Hinsdale data
% ------------------------------------------------------------------------
net = resetState(net);
num_input_steps = 200;
test_traj_num = 1;
num_time_steps = height(test_data_table{test_traj_num});
[net, preds] = predictAndUpdateState(net, test_data_array{test_traj_num}(:,1:num_input_steps));

% get last prediction
roll_preds = preds(1:des_pred_steps, des_pred_steps);
pitch_preds = preds(des_pred_steps+1:2*des_pred_steps, des_pred_steps);

% get imu ground truth 
imu_roll = test_outputs_array{test_traj_num}(1:des_pred_steps,test_traj_num);
imu_pitch = test_outputs_array{test_traj_num}(des_pred_steps+1:-1,test_traj_num);

% Unnormalize (so in radians)
unnorm_roll_preds = (roll_preds - p.mu(1))*p.sig(1);
unnorm_pitch_preds = (pitch_preds - p.mu(2))*p.sig(2);
unnorm_imu_roll = (imu_roll - p.mu(1))*p.sig(1);
unnorm_imu_pitch = (imu_pitch - p.mu(2))*p.sig(2);

figure
t = tiledlayout(2, 1);
title("Network Predictions")

nexttile
plot(1:num_time_steps, ((test_data_table{test_traj_num}{:,"roll"}-p.mu(1)).*p.sig(1)))
hold on
plot(num_input_steps+1:num_input_steps+des_pred_steps, unnorm_roll_preds)
legend("IMU", "Prediction")
ylabel("Roll (radians)")
xlabel("Time step")

nexttile
plot(1:num_time_steps, ((test_data_table{test_traj_num}{:,"pitch"}-p.mu(2)).*p.sig(2)))
hold on
plot(num_input_steps+1:num_input_steps+des_pred_steps, unnorm_pitch_preds)
legend("IMU", "Prediction")
ylabel("Pitch (radians)")
xlabel("Time step")
%%
% ------------------------------------------------------------------------
%                                Functions
% ------------------------------------------------------------------------
function init_options = define_new_opts(val_inputs, val_outputs)
    init_options = trainingOptions("adam", ...
        InitialLearnRate=0.001,...
        LearnRateDropPeriod=5, ...
        LearnRateSchedule='piecewise', ...
        LearnRateDropFactor=.8, ...
        MaxEpochs = 5, ...
        MiniBatchSize=16, ...
        SequencePaddingDirection="right", ...
        Plots="none", ...
        Shuffle='every-epoch', ...
        ValidationData={val_inputs, val_outputs}, ...
        ValidationFrequency = 60, ...
        OutputNetwork='best-validation-loss', ...
        ExecutionEnvironment='gpu');
end


