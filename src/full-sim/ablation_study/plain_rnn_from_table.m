%{
Trains a neural network on training data from Julia, all saved as tables. 

Assumes that data is input at 25 Hz.

Make sure data input is the correct one, and that MaxEpochs is set to 50 in
define_new_opts. 

Last modified 9/12/23
%}
% Load in the data
load("data/full-sim-data-091023/NormalizedTableTrainDataNoisy.mat")
train_data_table = sequence_data_table;
load("data/full-sim-data-091023/NormalizedTableValDataNoisy.mat")
val_data_table = sequence_data_table;
load("data/full-sim-data-091023/NormalizedTableTestData.mat")
test_data_table = sequence_data_table;

load("data/full-sim-data-091023/normparams.mat")

% define constants from the data
des_pred_length = 2.; % seconds
% sim_dt = train_data_table{1}{2,"time_secs"} - train_data_table{1}{1, "time_secs"};
sim_dt = .04;
des_pred_steps = round(des_pred_length/sim_dt);

% which columns are we attempting to predict?
gt_pitch_column_name = "qs2";
gt_roll_column_name = "qs1";
% input_data_columns = ["qs1", "qs2", "qs7", "qs8", "qs9", "qs10", "vs1", "vs2", "vs3", "vs7", "vs8", "vs9", "vs10"];
input_data_columns = ["noisy_qs1", "noisy_qs2", "noisy_qs7", "noisy_qs8", "noisy_qs9", "noisy_qs10", "noisy_vs1", "noisy_vs2", "noisy_vs3", "noisy_vs7", "noisy_vs8", "noisy_vs9", "noisy_vs10"];

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
disp("Formatting training data...")
for n = 1:numel(train_data_table)
    gt_data{n} = array2table(zeros(height(train_data_table{n})-des_pred_steps, des_pred_steps*2));
    sequence_length = height(train_data_table{n})-des_pred_steps;
    for i = 1:sequence_length
        gt_data{n}{i,1:des_pred_steps} = train_data_table{n}{i+1:i+des_pred_steps, gt_roll_column_name}';
        gt_data{n}{i,des_pred_steps+1:end} = train_data_table{n}{i+1:i+des_pred_steps, gt_pitch_column_name}';
    end
    train_data_table{n} = train_data_table{n}(:,input_data_columns);
    train_data_array{n} = table2array(train_data_table{n}(1:sequence_length,:))';
    gt_array{n} = table2array(gt_data{n})';
end
disp("Formatting validation data...")
% Format validation data
for n = 1:numel(val_data_table)
    val_outputs_table{n} = array2table(zeros(height(val_data_table{n})-des_pred_steps, des_pred_steps*2));
    sequence_length = height(val_data_table{n})-des_pred_steps;
    for i = 1:sequence_length
        val_outputs_table{n}{i,1:des_pred_steps} = val_data_table{n}{i+1:i+des_pred_steps, gt_roll_column_name}';
        val_outputs_table{n}{i,des_pred_steps+1:end} = val_data_table{n}{i+1:i+des_pred_steps, gt_pitch_column_name}';
    end
    val_data_table{n} = val_data_table{n}(:,input_data_columns);
    val_data_array{n} = table2array(val_data_table{n}(1:sequence_length,:))';
    val_outputs_array{n} = table2array(val_outputs_table{n})';
end
%%
disp("Formatting test data...")
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
close all

rmse_tracker = {};
rmse_tracker_arrays = [];


bad_trajs = [9, 29, 30, 34, 35, 36, 46];

i = 40
good_it = 1;
% for i = 1:57
% for pred_start_time_secs = 5:.4:12
    if ~ismember(i, bad_trajs)
        net = resetState(net);
        pred_start_time_secs = 7.4;
        num_input_steps = round(pred_start_time_secs/sim_dt);
        test_traj_num = i;
        num_time_steps = height(test_data_table{test_traj_num});
        [net, preds] = predictAndUpdateState(net, test_data_array{test_traj_num}(:,1:num_input_steps));
        
        net = resetState(net);
        [net, whole_traj_preds] = predictAndUpdateState(net, test_data_array{test_traj_num}(:,1:num_time_steps-des_pred_steps-1));
        
        % separate predicted rolls and pitches
        preds_rolls_rad = whole_traj_preds(1:des_pred_steps, :)*p.sig(1)+p.mu(1);
        preds_pitches_rad = whole_traj_preds(des_pred_steps+1:des_pred_steps*2,:)*p.sig(2)+p.mu(2);
        preds_rolls_deg = rad2deg(preds_rolls_rad);
        preds_pitches_deg = rad2deg(preds_pitches_rad);
        
        % get last prediction (degrees)
        preds_roll_here = preds_rolls_deg(:,num_input_steps);
        preds_pitches_here = preds_pitches_deg(:,num_input_steps);
        
        % get imu ground truth 
        gt_rolls_normed = test_outputs_array{test_traj_num}(1:des_pred_steps,1:num_time_steps-des_pred_steps-1);
        gt_pitches_normed = test_outputs_array{test_traj_num}(des_pred_steps+1:des_pred_steps*2,1:num_time_steps-des_pred_steps-1);
        gt_rolls_rad = gt_rolls_normed*p.sig(1)+p.mu(1);
        gt_pitches_rad = gt_pitches_normed*p.sig(2)+p.mu(2);
        gt_rolls_deg = rad2deg(gt_rolls_rad);
        gt_pitches_deg = rad2deg(gt_pitches_rad);
        
        % imu ground truth for this particular spot
        gt_rolls_here = gt_rolls_deg(:,num_input_steps);
        gt_pitches_here = gt_pitches_deg(:,num_input_steps);
        
        % figure
        t = tiledlayout("horizontal", "TileSpacing","tight")
        % title(t, "Prediction Comparison")
        sim_times = sim_dt:sim_dt:sim_dt*num_time_steps;
        pred_times = (num_input_steps+1)*sim_dt : sim_dt : (num_input_steps+des_pred_steps)*sim_dt;

        nexttile
        plot(sim_times, rad2deg(test_data_table{test_traj_num}{:,"roll"}*p.sig(1)+p.mu(1)), 'Color', '#DDCC77', 'LineWidth', 2)
        hold on
        plot(pred_times, preds_roll_here, 'Color', '#0072B2', 'LineWidth', 2)
        plot(pred_times, const_ori_table{test_traj_num}{num_input_steps,1:50}, '--', 'Color', '#56B4E9', 'LineWidth', 2)
        plot(pred_times, const_vel_table{test_traj_num}{num_input_steps,1:50}, '-.', 'Color', '#CC6677', 'LineWidth', 2)
        xlim([6.5,10.5])
        % legend("IMU", "RNN", "Const Ori", "Const Vel")
        ylabel("Roll (degrees)")
        xlabel("Time (s)")
        hold off
        set(gca, ...
          'Box'         , 'off'     , ...
          'TickDir'     , 'out'     , ...
          'TickLength'  , [.02 .02] , ...
          'XMinorTick'  , 'on'      , ...
          'YMinorTick'  , 'off'      , ...
          'YGrid'       , 'on'      , ...
          'YTick'       , [-12:2:8], ...
          'XGrid'       , 'off'     , ...
          'XColor'      , [.3 .3 .3], ...
          'YColor'      , [.3 .3 .3], ...
          'LineWidth'   , 1         );

        nexttile
        plot(sim_times, rad2deg(test_data_table{test_traj_num}{:,"pitch"}*p.sig(2)+p.mu(2)), 'Color', '#DDCC77', 'LineWidth', 2)
        hold on
        plot(pred_times, preds_pitches_here, 'Color', '#0072B2', 'LineWidth', 2)
        plot(pred_times, const_ori_table{test_traj_num}{num_input_steps, 51:100}, '--', 'Color', '#56B4E9', 'LineWidth', 2)
        plot(pred_times, const_vel_table{test_traj_num}{num_input_steps, 51:100},  '-.', 'Color', '#CC6677', 'LineWidth', 2)
        xlim([6.5,10.5])
        lgd = legend("Ground truth", "RNN", "Const. ori", "Const. vel");
        lgd.Location = 'eastoutside';
        ylabel("Pitch (degrees)")
        xlabel("Time (s)")
            set(gca, ...
              'Box'         , 'off'     , ...
              'TickDir'     , 'out'     , ...
              'TickLength'  , [.02 .02] , ...
              'XMinorTick'  , 'on'      , ...
              'YMinorTick'  , 'off'      , ...
              'YGrid'       , 'on'      , ...
              'YTick'       , [-15:1:8], ...
              'XGrid'       , 'off'     , ...
              'XColor'      , [.3 .3 .3], ...
              'YColor'      , [.3 .3 .3], ...
              'LineWidth'   , 1         );

        set(gcf, ...
            'Position', [100 100 650 225]);
        
        % disp("==========")
        % rmse_table = array2table(zeros(2,3));
        % rmse_table.Properties.VariableNames = {'RNN', 'Const Angle', 'Const Vel'};
        % rmse_table.Properties.RowNames = {'Roll', 'Pitch'};
        % rmse_table{'Roll', 'RNN'} = rmse(gt_rolls_deg, preds_rolls_deg, "all");
        % rmse_table{'Pitch', 'RNN'} = rmse(gt_pitches_deg, preds_pitches_deg, "all"); 
        % rmse_table{'Roll', 'Const Angle'} = rmse(gt_rolls_deg, const_ori_table{test_traj_num}{2:end,1:50}', "all");
        % rmse_table{'Pitch', 'Const Angle'} = rmse(gt_pitches_deg, const_ori_table{test_traj_num}{2:end,51:100}', "all");
        % rmse_table{'Roll', 'Const Vel'} = rmse(gt_rolls_deg, const_vel_table{test_traj_num}{2:end,1:50}', "all");
        % rmse_table{'Pitch', 'Const Vel'} = rmse(gt_pitches_deg, const_vel_table{test_traj_num}{2:end,51:100}', "all")
        % 
        % rmse_tracker{good_it} = rmse_table;
        % rmse_tracker_arrays(:,:,good_it) = table2array(rmse_table);
        % good_it = good_it + 1;
    end
% end

% summary_table = array2table(mean(rmse_tracker_arrays,3));
% summary_table.Properties.VariableNames = {'RNN', 'Const Angle', 'Const Vel'};
% summary_table.Properties.RowNames = {'Roll', 'Pitch'}


%%
% ----- zero velocity assumption -----
const_ori_table = test_outputs_table;
const_vel_table = test_outputs_table;

for test_traj_num = 1:57
    for n = 1:height(test_outputs_table{test_traj_num})
        % get starting values
        starting_roll_rads = test_data_table{test_traj_num}{n,"roll"}*p.sig(1)+p.mu(1);
        starting_pitch_rads = test_data_table{test_traj_num}{n,"pitch"}*p.sig(2)+p.mu(2);
        starting_x_angvel = test_data_table{test_traj_num}{n,"x_angvel"}*p.sig(7)+p.mu(7);
        starting_y_angvel = test_data_table{test_traj_num}{n, "y_angvel"}*p.sig(8)+p.mu(8);

        % how much roll and pitch will change per time step
        d_roll_dt = starting_x_angvel*sim_dt;
        d_pitch_dt = starting_y_angvel*sim_dt;
        
        const_ori_table{test_traj_num}{n,1:50} = rad2deg(starting_roll_rads);
        const_ori_table{test_traj_num}{n,51:100} = rad2deg(starting_pitch_rads);

        const_vel_table{test_traj_num}{n,1:50} = rad2deg(starting_roll_rads+d_roll_dt*(1:50));
        const_vel_table{test_traj_num}{n,51:100} = rad2deg(starting_pitch_rads+d_pitch_dt*(1:50));
    end
end

% ------------------------------------------------------------------------
%                                Functions
% ------------------------------------------------------------------------
function init_options = define_new_opts(val_inputs, val_outputs)
    init_options = trainingOptions("adam", ...
        InitialLearnRate=0.001,...
        LearnRateDropPeriod=5, ...
        LearnRateSchedule='piecewise', ...
        LearnRateDropFactor=.8, ...
        MaxEpochs = 50, ...
        MiniBatchSize=16, ...
        SequencePaddingDirection="right", ...
        Plots="none", ...
        Shuffle='every-epoch', ...
        ValidationData={val_inputs, val_outputs}, ...
        ValidationFrequency = 60, ...
        OutputNetwork='best-validation-loss', ...
        ExecutionEnvironment='gpu');
end


