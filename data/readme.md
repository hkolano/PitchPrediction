# Data

```toy-data``` holds the first round of toy problem data. It has 6 columns: {Pitch position, Joint 1 position, Joint 2 position} and then velocities. All trajectories start from equilibrium and go to one waypoint with a quintic trajectory at a certain time scaling. ```toy-data-waypoints.csv``` has the waypoint data associated with those trajectories. 

```toy-data-matlab/TestandTrainData_071322.mat``` holds the same data as ```toy-data```, but in .mat format for network training. It was generated with ```train_toy_data.m```. 

```toy-data-with-desv``` adds two more columns for the desired velocities of the arm joints. 