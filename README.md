# TSP Solver Robot with ROS2 and Gazebo

This project implements an autonomous robot that solves the Traveling Salesman Problem (TSP) in a maze environment using ROS2 Jazzy and Gazebo Harmonic. The robot uses simulated annealing optimization combined with A* pathfinding to efficiently navigate through waypoints in a procedurally generated maze.

## Key Features

- **Procedural maze generation** with configurable dimensions
- **TSP optimization** using simulated annealing algorithm
- **A star pathfinding** for collision-free navigation between waypoints
- **Autonomous navigation** in Gazebo simulation environment
- **ROS2-Gazebo bridge** for seamless communication

## File Descriptions

### `maze_generator_sdf`
Generates a customizable maze with dimensions of X rows by Y columns, where each cell is 2m × 2m. Creates visual start and end markers, and spawns the robot at a random available position within the maze.

### `a_star_path.py`
Provides the core A* search algorithm implementation. Exports the `a_star_search()` function that calculates optimal collision-free paths between any two points in the maze, used by other navigation scripts.

### `simulated_annealing.py`
Implements the simulated annealing optimization algorithm to solve the TSP. Calculates the optimal order for visiting all waypoints in the maze, minimizing total travel distance.

### `path_follower.py`
Robot navigation controller that executes the planned path in the Gazebo environment. Receives waypoints from `simulated_annealing.py` and controls the robot's movement to follow the calculated trajectory.

### `setup_bridges.sh`
Bash script that initializes and configures the ROS2-Gazebo bridges, enabling communication between the ROS2 control system and the Gazebo simulation environment.

### `useful_functions.py`
Utility module containing helper functions used across multiple scripts for common operations

## Implementation
First, I designed and modeled a differential drive robot from scratch for the Gazebo Harmonic simulation environment.
<br/><br/>
When I had the robot, it was time to create the environment. I implemented a procedural maze generation system based on a recursive backtracking algorithm to create unique, solvable mazes for each simulation run. I configured the system to generate 9x9 grid mazes where each cell is 2 meters wide, resulting in an 18x18 meter navigable space with 1-meter-high walls. The algorithm guarantees that every empty space in the maze is reachable, ensuring valid paths always exist between any two points. I also spawn colored ground markers - a green cylinder at the start point and a red cylinder at the end point. The robot is automatically placed at the start position when the world loads. Additionally, I export a JSON configuration file for each maze containing critical metadata: the occupancy grid (for path planning algorithms), start and end coordinates in both grid and world frames, and maze parameters like dimensions and the random seed used for generation.
<br><br>
In the following week, I focused on developing a ROS2 node that controls the robot to autonomously navigate through a sequence of waypoints in the maze. I implemented a two-state controller with a "ROTATE" state for orienting toward the target and a "MOVE" state for driving forward while maintaining heading. The system uses proportional control for both linear and angular velocities, dynamically adjusting speed based on distance to target and angle error. I added course correction that switches back to rotation mode if the robot drifts too far off heading during movement.
Once I achieved seamless and precise navigation between random points, I implemented the A* pathfinding algorithm to calculate optimal collision-free paths between any two points in the maze. The algorithm explores in four directions (north, south, east, west).
<br><br>
In the subsequent weeks, I focused on implementing a simulated annealing algorithm to solve the Traveling Salesman Problem and find the optimal order to visit all waypoints in the maze. The system first computes a distance matrix between all points using A* pathfinding to get actual traversable distances, ensuring the solution accounts for walls. I used a nearest-neighbor heuristic to generate a good initial solution, then applied simulated annealing. I integrated the system to work with mandatory start and finish points, ensuring the robot begins at the green marker and ends at the red marker while visiting all intermediate waypoints optimally.
<br><br>
I also implemented a visualization system using matplotlib animations that shows the maze layout, all waypoints, the full path, and an animated line that traces the robot's planned trajectory from start to finish.

## Example generated mazes
### Maze 9x9
<img width="647" height="627" alt="maze9x9" src="https://github.com/user-attachments/assets/eee0d752-1f2a-4f18-b798-fbbbd89a8a3c" />


### Maze 17x17
<img width="621" height="620" alt="maze17x17" src="https://github.com/user-attachments/assets/89ac2af5-eb19-4111-9ad4-d07f918bab8c" />


### Maze 25x25
<img width="841" height="831" alt="maze25x25" src="https://github.com/user-attachments/assets/73b8c0f6-0fd4-42b4-941b-afc759fd00cc" />


## TSP visualization
### SA solution (S = 62 meters)
https://github.com/user-attachments/assets/3889d9ba-d7d0-41e2-869b-84523ff5b62d

### Held-Karp solution (S =  58 meters)
https://github.com/user-attachments/assets/7cc036dd-8817-4c37-a663-9691cbdd1bb6


## Gazebo simulation
https://github.com/user-attachments/assets/e856a067-3093-4311-9b94-f473b074fc70
