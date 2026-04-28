# Quasi-Static-Model
Quasi Static model evolved from "Quasi-Static-Integrative-Model"

How to Run:
1. Open `main.m` from the MATLAB IDE and run. **NOTE**: this will clear the workspace.
2. Select the dataset(s) for the analysis in the popup. 

    This will output a collection of datasets called `Fly_Master` to the workspace. Each dataset within `Fly_Master` will be saved to their respective output directory: `Data_Sets/[dataset name]/Outputs/`

    Each new output dataset will contain a `Dynamics` structure that holds the computed forces and torques. 

3. Open any of the programs in `Plots/` and run to visualize the data. **NOTE**: Some plots are currently work in progress.