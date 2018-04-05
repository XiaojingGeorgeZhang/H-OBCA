# H-OBCA
Hierarchical Optimization-Based Collision Avoidance - An algorithm for generating dynamically feasible parking trajectories 

Paper describing the theory can be found [here](http://arxiv.org/abs/1711.03449).

## Short Description
H-OBCA is an optimization-based  approach  for autonomous  parking. It builds on [OBCA](https://github.com/XiaojingGeorgeZhang/OBCA), which is a recent method for generating obstacle-free trajectories using optimal control.

H-OBCA is able to generate high-quality *kino-dynamically feasible obstacle-free* trajectories. These trajectories are smooth, and can be accurately tracked by simple low-level path following controllers. A [Julia](https://julialang.org/)-based implementation is provided.


## Examples

### H-OBCA for Reverse Parking
<img src="https://github.com/XiaojingGeorgeZhang/H-OBCA/blob/master/images/TrajReverseHOBCA.gif" width="700" />

### H-OBCA for Parallel Parking
<img src="https://github.com/XiaojingGeorgeZhang/H-OBCA/blob/master/images/TrajParallelHOBCA.gif" width="700" />


## How to run the Parking code:

### First steps

1. Change to the directory

2. Install Julia from https://julialang.org/downloads/ (code tested on version 0.5.1 and 0.6.0) 

3. Open Julia in terminal

4. Install Julia package JuMP using Pkg.add("JuMP")

5. Install Julia package Ipopt using Pkg.add("Ipopt")

6. Install Julia package PyPlot using Pkg.add("PyPlot")

7. Install Julia package NearestNeighbors using Pkg.add("NearestNeighbors")

8. Install Julia package ControlSystems using Pkg.add("ControlSystems")


### Running the parking example 

1. Start Julia in terminal

2. Type in terminal: include("setup.jl")

3. Type in terminal: include("main.jl")


### modifying the code 

1. To play with start points, change x0 in main.jl and run 
the code by: include("main.jl")

2. If you change anything in one of the collision avoidance
problems, you need to activate the changes by running:
include("setup.jl")

3. If you change the size of the car in main.jl, the change 
also need to be made in collision_check.jl

### Note
1. this code has been tested on Julia 0.5.1 and 0.6.0, and might not run on any other Julia versions

2. For best results, run code in Julia terminal
