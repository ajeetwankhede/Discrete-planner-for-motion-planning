# Discrete-planner-for-motion-planning

[![Build Status](https://travis-ci.com/ajeetwankhede/Discrete-planner-for-motion-planning.svg?branch=master)](https://travis-ci.com/ajeetwankhede/Discrete-planner-for-motion-planning)
---

## Project Overview
2D discrete path planners are designed for the navigation of a holonomic robot in a known static and flat environment. All the planners share the same common interface and should implement the ​search​ method that has the following signature:
search(world_state, robot_pose, goal_pose) return path
Inputs:
world_state is a 2D-grid representation of the environment where the value 0 indicates a navigable space and the value 1 indicates an occupied/obstacle space.
robot_pose is a tuple of two indices (x, y) which represent the current pose of the robot in world_state​.
goal_pose​ ​is a tuple of two indices (x, y) which represent the goal in ​world_state coordinate system.
Output:
path is a list of tuple (x, y) representing a path from the ​robot_pose to the goal_pose​ in world_state​ or​ None​ if no path has been found.
Two discrete planners are implemented:

1. Random Planner
The random planner tries to find a path to the goal by randomly moving in the environment (only orthogonal moves are legal). If the planner can not find an acceptable solution in less than max_step_number, the search should fail. The random planner, while being erratic, has a short memory, and it will never attempt to visit a cell that was visited in the last ​sqrt(max_step_number)​ steps except if this is the only available option.

2. Optimal Planner
A 2D path planner with A* algorithm is designed and developed, for the navigation of the robot (only orthogonal moves are legal). The A* algorithm ensures optimality of the path, with obstacle avoidance defined in the world.

The performance of these two planners is compared on basis of search space explored, time taken to find a path, and optimality of the path found. Random planner was run 10 times and optimal planner once, and the results are summarized in the following table.

## Link for SIP document
[SIP Document]https://docs.google.com/spreadsheets/d/1OKOs_5UbBNU4q0WjCL7nKXChtRCFgHgl8hptJVIeyFo/edit?usp=sharing

## Dependencies

The path planning module has following dependencies:
1. googletest
2. cmake
3. gnuplot
4. [gnuplot-iostream](http://stahlke.org/dan/gnuplot-iostream/)

## Standard install via command-line
```
git clone --recursive https://github.com/ajeetwankhede/Discrete-planner-for-motion-planning
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app
```

## Run a demo
After running the steps for standard install via comman-line, run the program. The program asks whether the user wants to run a demo. Type 'm' for a demo and then select any planner. 
The demo example should generate following figures.

1. Random planner demo:

<p align="center">
<img src="/output/random-planner-output.png">
</p>

2. Optimal planner demo:

<p align="center">
<img src="/output/optimal-planner-output.png">
</p>

## Building for code coverage
The code coverage of the functions is 82.1% and to get a .html output run the following commands.
```
sudo apt-get install lcov
cd <path to repository>
mkdir build
cd build
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```
This generates a index.html page in the build/coverage sub-directory that can be viewed locally in a web browser.
Screenshot of the .html file is shown below.
 
<p align="center">
<img src="/output/Code Coverage.png">
</p>

## How to generate Doxygen report

```
sudo apt-get install doxygen
cd <path to repository>
mkdir Doxygen
cd Doxygen
doxygen -g <config_file_name>
gedit <config_file_name>
```
Update PROJECT_NAME and INPUT fields in the configuration file.
Then run the following command to generate the documentations.
```
doxygen <config_file_name>
```
In Doxygen folder, config file and genertaed reports are saved.
