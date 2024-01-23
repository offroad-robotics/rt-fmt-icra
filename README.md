# Real-Time Fast Marching Tree (RT-FMT) for Mobile Robot Motion Planning in Dynamic Environments (ICRA 2023)

## Overview
This repository contains the implementation of the Real-Time Fast Marching Tree (RT-FMT) algorithm, as proposed in the paper titled "[Real-Time Fast Marching Tree (RT-FMT) for Mobile Robot Motion Planning in Dynamic Environments](https://ieeexplore.ieee.org/document/10160595)". The algorithm is designed for robotic motion planning, featuring local and global path generation, multiple-query planning, and dynamic obstacle avoidance. 

## Simulation Results:
The repository includes simulation code demonstrating the superior performance of RT-FMT compared to RT-RRT* in terms of execution cost and arrival time in most scenarios. The simulations also showcase the benefits of taking a local path before the global path is available, emphasizing the algorithm's capability to reduce arrival time even when faced with the possibility of selecting an inferior path.

## Getting started
- Install unity hub, then open then add the cloned repo rt-fmt-icra from disk. It will ask you to install unity 2018.1.0f2
- Once you open the imported project, you will find all scenarios under the Assets/Scenes folder
- Some example scenarios are available to visualize the algorithm and the experiments folder contain the scenarios used in the paper

## Code Structure

The RT-FMT implementation is organized into the following key components:

### `src` Directory

- `algorithm/`: Contains the core implementation of the RT-FMT algorithm.
- `visualization/`: Includes code related to visualizing simulation results.
- `utilities/`: Houses utility functions used across the project.

### `Assets` Directory

- `Scenes/`: Contains Unity scenes representing different simulation scenarios.
- `Scripts/`: Holds scripts that interact with the RT-FMT algorithm.

### `Experiments` Directory

- `Scenario1/`: Specific scenarios used in the paper for experimentation.
- `Scenario2/`: ...

### File Descriptions

#### `algorithm/RTFMT.cs`

This file implements the RT-FMT algorithm, including the core planning logic and tree generation.

#### `visualization/Visualizer.cs`

The Visualizer script handles the visualization of simulation results, providing insights into the algorithm's behavior.

#### `Scripts/RobotController.cs`

The RobotController script manages the movement and behavior of the robot in the Unity simulation.
