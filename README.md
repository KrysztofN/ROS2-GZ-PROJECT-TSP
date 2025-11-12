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
In the following week, I focused on developing a ROS2 node that controls the robot to autonomously navigate through a sequence of waypoints in the maze. I implemented a two-state controller with a "ROTATE" state for orienting toward the target and a "MOVE" state for driving forward while maintaining heading. The system uses proportional control for both linear and angular velocities, dynamically adjusting speed based on distance to target and angle error. I added intelligent course correction that switches back to rotation mode if the robot drifts too far off heading during movement.
Once I achieved seamless and precise navigation between random points, I implemented the A* pathfinding algorithm to calculate optimal collision-free paths between any two points in the maze. The algorithm explores in four cardinal directions (north, south, east, west).
<br><br>
In the subsequent weeks, I focused on implementing a simulated annealing algorithm to solve the Traveling Salesman Problem and find the optimal order to visit all waypoints in the maze. The system first computes a distance matrix between all points using A* pathfinding to get actual traversable distances rather than Euclidean distances, ensuring the solution accounts for walls and obstacles. I used a nearest-neighbor heuristic to generate a good initial solution, then applied simulated annealing. I integrated the system to work with mandatory start and finish points, ensuring the robot begins at the green marker and ends at the red marker while visiting all intermediate waypoints optimally.
<br><br>
I also implemented a visualization system using matplotlib animations that shows the maze layout, all waypoints, the full path, and an animated line that traces the robot's planned trajectory from start to finish.

## Showcase
### Maze 5x5
![Maze 5x5](src/images/5x5.png)

### Maze 9x9
![Maze 9x9](src/images/9x9.png)

### Maze 15x15
![Maze 15x15](src/images/15x15.png)
