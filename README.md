# Quasi-Steady-Model
Quasi-steady model evolved from "Quasi-Static-Integrative-Model".

## Description
This MATLAB module calculates the net forces and torques that _Drosophila melanogaster_ (common fruitfly) generates from flapping its wings. Variable wing damage is also supported.

## Instructions
1. (Optional) Upload your own dataset into `Data_Sets`.

    This module is preloaded with kinematic datasets from two experiments. The datasets prefixed with `fly_` correspond to a tethered fly that gets cut mid-flight (Fry et al. 2005) and `Robot_` relates to a dynamically-scaled robotic fly with various wingbeat patterns and wing damage (Muijres et al. 2017). Use these datasets as references to format your own dataset.
2. Open `main.m` from the MATLAB IDE and run. 

    **NOTE**: this will clear the workspace.
3. Select the dataset(s) for the analysis in the popup. 

    This will output a collection of datasets called `Fly_Master` to the workspace. Each dataset within `Fly_Master` will be saved to their respective output directory: `Data_Sets/[dataset name]/Outputs/`. Each new output dataset will contain a `Dynamics` structure that holds the computed forces and torques. 
4. Call `import Plots.*` to import all the plotting scripts, or choose a specific one.
5. Run the plotting script(s).
    
    **NOTE**: Some combination of scripts and datasets may be incompatible. Some plots are currently work in progress.

## Dataset Tree Example
```
Data_Sets
├── Example_Dataset
.   ├── Inputs
.   │   ├── Fly_Data.mat
.   │   └── Kinematics.mat
    └── Outputs
```

## TODO Tracker
| File | Line No. | Description |
| --- | :---: | --- |
| main.m | 8 | before removing this, remove global variables and investigate duplicate points in delaunayTriangulation |
| main.m | 51 | Investigate the purpose of this operation |
| main.m | 75 | move this into Analysis.m |
| +Utils/Analysis.m | 47 | Rename Kin to something descriptive and unambiguous |
| +Utils/Analysis.m | 72 | Rewrite into a proper Dynamics constructor |
| +Utils/Analysis.m | 84 | Unpack Fly, every Fly_Master element should just be a struct of Kinematics, Morphology, Dynamics, time, Fly_Num, and Attributes |

## References
​F. T. Muijres, N. A. Iwasaki, M. J. Elzinga, J. M. Melis, and M. H. Dickinson,​ ​"Flies Compensate for Unilateral Wing Damage Through Modular Adjustments of Wing​ ​and Body Kinematics," Interface Focus, vol. 7, no. 1, p. 20160103​, Feb. 2017, doi: 10.1098/rsfs.2016.0103.

S. N. Fry, R. Sayaman, and M. H. Dickinson, "The Aerodynamics of Hovering Flight in Drosophila," Journal of Experimental Biology, vol. 208, no. 12, pp. 2303–2318, Jun. 2005, doi: 10.1242/jeb.01612.
