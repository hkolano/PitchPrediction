# Full Simulation

This folder contains all the scripts for running pitch prediction with all 10 degrees of freedom involved. 

## Data Generation

To generate trajectory data, run data-generation/MainSim.jl; make sure to set the number of trajectories to a high number. Also change the folder name to save the trajectory files to. Each trajectory will save to a separate csv file in the indicated folder. 

The variable goal_freq in MainSim.jl determines the frequency at which the outputs are saved to the CSV file. For example, if goal_freq = 100, data will be output at 100 Hz. 50Hz is the highest frequency used for the neural network, so 50Hz should be the default.

## Data Parsing
After the data has been generated, run import_and_parsing/import_full_data_meas_and_actual.m to import it into a MATLAB file. Make sure to change which folder contains the trajectory data and which folder to save the MATLAB file to. 

Run helper_funcs/define_channel_idxs_act_and_meas_inputs.m to define which table indices belong to which data stream groups. Make sure to indicate which folder to save the file to. 

If the data was output at a value other than 50Hz, use import_and_parsing.m to create a downsampled version. Additionally, use this script to generate 10Hz data for the k length study. 

## Ablation Study
ablation_study/ablation_study_benchmark does three runs of the neural network with all data streams included. ablation_study/ablation_study_recursive runs the rest of the study. It runs the study with n-1 data streams, removes the one with the lowest validation RMSE, and so on until only 2 groups remain. The variable 'all_RMSES' output from that file should contain the values to be plotted. ablation_study/plot_ablation_results_manual_input.m should be used to plot the results (after manually inputting the resulting RMSE values). 

## K Length Study
This study has two separate sections: making pitch-only prediction networks and autoregressive networks. After both sets of networks are generated, a validation set is used to evaluate their accuracy.

### Pitch Only Networks

### Autoregressive Networks

### Validation and Plotting

