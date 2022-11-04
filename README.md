# jaco2-stroke-rehab-sim
## About this project
This project was made by second semester students from Aalborg University, studying robotics engineering. The project subject was 'robotic manipulator programming & simulation', which includes and requires the project to do all the math and programming for inverse, and forward kinematics as well as trajectory planning without the usage of libraries. The bare necessity libraries are excused.

## Introduction
In our approach to the subject we were given, we agreed upon coding and simulating inverse, and forward kinematics as well as trajectory planning for the Kinova Jaco2.
This is done without the usage of external libraries, per guidelines by Aalborg University. The necessary libraries for simulating a Kinova Jaco2 accurately was excused.
In this project a lot of the library code used is from ([Peter Corke's Robotics Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/ "Peter Corke's Robotics Toolbox")).

The project focuses on rehabilitation for stroke patients with motoric defects as a consequence of damage to the motoric parts of their brains. Activation of neuroplasticity stimulates the brain to rebuild the dead cells and thereby regain function, rehabilitation exercises target the disability by
moving the disabled body part. Rehabilitation exercises are repetitive to stimulate the area of the damage and the surrounding brain cells to regain the disabled/ damaged function. Rehabilitation consists of body movements that the patient actively or inactively completes, thereby improving muscles and neuron connectivity to the area.

This is essentially what the final product that this project simulates is supposed to do!

## Features
The code found in this repository is not only restricted to usage relevant to rehabilitation for stroke patients. It can generally be used to simulate inverse, and forward kinematics in any way, alongside features like trajectory planning, where this thing comes with plotting and data outputs during runtime. The code can generate animations of the simulation and can save animations in formats like .GIF and .MP4, .PNG & more. (Note that .PNG, and .JPG as two examples, will make the code export every frame as an individual .PNG/.JPG file).

## Examples / Demos
This repository comes with a few examples in the examples folder, but let's go through it while you are here.

### Trajectory Movement Simulation
Exported .GIF animations of the simulation (Please note this only shows blue lines for manipulator links) in different directions:                   
![Moving from left to right](https://raw.githubusercontent.com/NikoBK/jaco2-stroke-rehab-sim/main/examples/robotRender/jacoLeftToRightLinear.gif)
![Moving from right to left](https://raw.githubusercontent.com/NikoBK/jaco2-stroke-rehab-sim/main/examples/robotRender/jacoRightToLeftLinear.gif)

The jitter that happens when on the second GIF showing movement from right to left, is not caused by the code per say, but rather the simulation, due to the inverse, and forward kinematics as well as trajectory being calculated in cartesian space.

### Trajectory Planning & Rendering
![Trajectory Frames](https://i.imgur.com/5JQHvVT.png "Trajectory Frames")     
Two different trajectory configurations. Left image: Linear 3-point trajectory. Right image: 5-point circular bow trajectory.

![Trajectory Frames 2](https://i.imgur.com/pvmFP88.png "Trajectory Frames 2")      
The same two trajectories from figure: 10.8, with blue trajectory frames indicating where the end-effector will move on the trajectory

![Trajectory Frames 3](https://i.imgur.com/dlkI8kA.png "Trajectory Frames 3")     
A zoomed in view of a early development trajectory, highlighting one of the via points with red color as well as the orientation towards XYZ.

### Joint Angle Plotting Over Time
The code can generate plotting over time that displays the angle in degrees to a specific joint at a specific time in space.
![left to right velocity](https://raw.githubusercontent.com/NikoBK/jaco2-stroke-rehab-sim/main/examples/jointAnglePlotting/linearLeftToRightAngles.png "left to right velocity")      
Shows the Joint angles over time for the Rightwards motion

![right to left velocity](https://raw.githubusercontent.com/NikoBK/jaco2-stroke-rehab-sim/main/examples/jointAnglePlotting/linearRightToLeftAngles.png "right to left velocity")     
Shows the Joint angles over time for the Leftwards motion

## What the robot actually looks like
The Kinova Jaco2 that is being simulated looks like this:
![real jaco2](https://robotnik.eu/wp-content/uploads/2020/05/robotnik-mico-3.jpg "Kinova Jaco2")   

The Kinova Jaco2 rendered in MATLAB:                    
![real jaco2](https://i.imgur.com/TcHu6CT.png "Kinova Jaco2")
