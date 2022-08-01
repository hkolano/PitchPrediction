# Toy UVMS Problem

## Workflow
1. Generate data in Julia with ```ToySim.jl```. It will be saved to n+1 csv files in ```data/toy-data```. 
2. Import data into matlab datastore with ```import_toy_data.m``` (formerly ```train_toy_data.m```). 
3. Train on one step predictions for one epoch with ```train_once.m```. 
4. Continue training with ```retrain_toy_data.m```. 

## Implementation Notes
In current configuration, the equilibrium position is [0.18558, -0.18558, 0]