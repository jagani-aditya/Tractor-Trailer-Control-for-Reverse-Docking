# Tractor-Trailer-Control-for-Reverse-Docking
A control algorithm is implemented to dock a tractor-trailer model in reverse


## Project Description
This project presents implementation of a control algorithm to park a tractor-trailer model in reverse for door-docking applications. We have used a LQR controller to track a trajectory. The trajectories are generated using a quintic polynomial function using initial and final positions of states.
The number of degrees of freedom are more than the number of states, hence, our system in underactuated.

## Demonstration
<p align="center">
  <img src="/images/model.PNG"  width = "80%" />
</p>

<p align="center">
  <img src="/images/info.PNG" width = "80%"/>
</p>

<p align="center">
  <img src="/images/reverse.png" width = "80%" />
</p>



## Platform
* MATLAB

## Implementation
 
Navigate to the ```MTT-System-GNN-Tracker``` folder

Open ```main.m``` file and ```RUN``` it in MATLAB workspace.

