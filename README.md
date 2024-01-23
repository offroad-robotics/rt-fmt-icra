# Real-Time Fast Marching Tree (RT-FMT) for Mobile Robot Motion Planning in Dynamic Environments (ICRA 2023)

## Overview
This repository contains the implementation of the Real-Time Fast Marching Tree (RT-FMT) algorithm, as proposed in the paper titled "[Real-Time Fast Marching Tree (RT-FMT) for Mobile Robot Motion Planning in Dynamic Environments](https://ieeexplore.ieee.org/document/10160595)". The algorithm is designed for robotic motion planning, featuring local and global path generation, multiple-query planning, and dynamic obstacle avoidance. 

## Simulation Results:
The repository includes simulation code demonstrating the superior performance of RT-FMT compared to RT-RRT* in terms of execution cost and arrival time in most scenarios. The simulations also showcase the benefits of taking a local path before the global path is available, emphasizing the algorithm's capability to reduce arrival time even when faced with the possibility of selecting an inferior path.


<p align="center">
  <img src="images/dynamic_example.gif" alt="Dynamic example" width="30%" style="margin-right: 10px;" />
  <img src="images/maze_example.gif" alt="Maze example" width="35%" style="margin-left: 10px;" />
</p>
<p align="left">
  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;     &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp; &nbsp;
  <em>Dynamic example</em>
  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  
  &nbsp;  &nbsp;  &nbsp;
  <em>Maze example</em>
</p>


## Getting started
- Install and open the unity hub software, then open a new project in the cloned repository folder. If you haven't done so It will ask you to install unity 2018.1.0f2
- Once you open the imported project, you will find all scenes under the `Assets/Scenes` folder, and the code related to the paper can be found in the `Assets/Scripts/RT-FMT` folder.
- The scenes contain different environments that can be used to visualize the code in `Scenes/Example` and to run the environments used in the paper, which are located at `Scenes/Experiments`

## Code Structure

The RT-FMT implementation is organized into the following key components:

### All RT-FMT code is located at the `Scripts/RT-FMT` Directory

- `RTFMTPlanner.cs`: Contains the core implementation of the RT-FMT algorithm.
- `RTFMT_example.cs`: Instantiates the planner and controls the sphere in the example scenes.
- `RTFMT_exp.cs`: Instantiates the planner and controls the sphere in the experiment scenes. This code has extra features to validate the experiment by running it multiple times and logging the results into a `.csv` file.



