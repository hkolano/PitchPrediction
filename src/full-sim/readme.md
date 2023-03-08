# Full Simulation

This folder contains all the scripts for running pitch prediction with all 10 degrees of freedom involved. 

## Data Generation

To generate trajectory data, run data-generation/MainSim.jl; make sure to set the number of trajectories to a high number. Also change the folder name to save the trajectory files to. Each trajectory will save to a separate csv file in the indicated folder. 

## Data Parsing
After the data has been generated, run import_and_parsing/import_full_data_meas_and_actual.m to import it into a MATLAB file. Make sure to change which folder contains the trajectory data and which folder to save the MATLAB file to. 

Run helper_funcs/define_channel_idxs_act_and_meas_inputs.m to define which table indices belong to which data stream groups. Make sure to indicate which folder to save the file to. 

## Ablation Study
