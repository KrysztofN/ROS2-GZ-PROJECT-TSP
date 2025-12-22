# TSP Solver Robot with ROS2 and Gazebo

This project implements an autonomous robot that solves the Traveling Salesman Problem (TSP) in a maze environment using ROS2 Jazzy and Gazebo Harmonic. The robot uses 2 methods for solving tsp (simulated annealing and Held-Karp) combined with A* pathfinding to efficiently navigate through waypoints in a procedurally generated maze. The robot needs to overcome few challanges along the way like respecting traffic lights and avoiding hitting objects. 

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

### `spawn_obstacles.py`
Ros2 node that spawns red and green markers (simulating traffic lights) in intervals as well as cones simulating road works or just obstacles on the road.

### `setup_bridges.sh`
Bash script that initializes and configures the ROS2-Gazebo bridges, enabling communication between the ROS2 control system and the Gazebo simulation environment.

### `useful_functions.py`
Utility module containing helper functions used across multiple scripts for common operations

## Implementation
First, I designed and modeled a differential drive robot from scratch for the Gazebo Harmonic simulation environment.
<br/><br/>
When I had the robot, it was time to create the environment. I implemented a procedural maze generation system based on a recursive backtracking algorithm to create unique, solvable mazes for each simulation run. I configured the system to generate nxn grid mazes where each cell is 2 meters wide. To make the maze more diverse and useful for showcasing the navigation possibilities I changed the script to randomly remove a given number of walls ence creating more interesting paths. I also spawn colored ground markers - a white cylinder at the start point and a black cylinder at the end point. The robot is automatically placed at the start position when the world loads. Additionally, I export a JSON configuration file for each maze containing critical metadata: the occupancy grid (for path planning algorithms), start and end coordinates in both grid and world frames, and maze parameters like dimensions and the random seed used for generation.
<br><br>
In the following week, I focused on developing a ROS2 node that controls the robot to autonomously navigate through a sequence of waypoints in the maze. I implemented a two-state controller with a "ROTATE" state for orienting toward the target and a "MOVE" state for driving forward while maintaining heading. The system uses proportional control for both linear and angular velocities, dynamically adjusting speed based on distance to target and angle error. I added course correction that switches back to rotation mode if the robot drifts too far off heading during movement.
Once I achieved seamless and precise navigation between random points, I implemented the A* pathfinding algorithm to calculate optimal collision-free paths between any two points in the maze. The algorithm explores in four directions (north, south, east, west).
<br><br>
In the subsequent weeks, I focused on implementing a simulated annealing algorithm to solve the Traveling Salesman Problem and find the optimal order to visit all waypoints in the maze. The system first computes a distance matrix between all points using A* pathfinding to get actual traversable distances, ensuring the solution accounts for walls. I used a nearest-neighbor heuristic to generate a good initial solution, then applied simulated annealing. I integrated the system to work with mandatory start and finish points, ensuring the robot begins at the green marker and ends at the red marker while visiting all intermediate waypoints optimally.
<br><br>
I also implemented a visualization system using matplotlib animations that shows the maze layout, all waypoints, the full path, and an animated line that traces the robot's planned trajectory from start to finish.
<br><br>
In the next phase I developed a ros2 node responsible for spawning markers and cones to simulate real-life street. The markers (red and green) act as traffic lights and cones act as obstacles signaling road maintenance.
<br><br>
Now I needed to update the node responsible for navigating the robot. The goal was to make the robot stop whenever it encounters a red light (robot already has in-built downward facing camera) and continues to move when a green light appears. I convert the camera readings into a hsv format and count pixels of desired color, if the condition is met I perform the desired action. The second thing was to implement a simple obstacle hit-avoidance. Apart from the camera, robot is also equipped with lidar, so the idea was to detect the obstacles and freeze until the obstacle is cleared out of the way. I set a threshold of 0.5m for detecting the obstacle and at each step check whether the lidar readings exceeded this threshold, and if they did the robot stops and resumes when the object is gone.
<br><br>
For the last changes I thought of adding different implementation of tsp solving algorithm. Apart from Simulated annealing I implemented Held-Karp algorithm which in contrast to SA is a dynamic-programming algorithm. I ran different mazes with different configurations and observed the paths outputted by SA and Held-Karp. The result was that Held-Karp was slighly better achieving shorter paths.
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


### Obstacle detection
https://github.com/user-attachments/assets/4bde4b0f-294f-4a2e-8894-29e1d0f496bd

### Traffic lights detection
https://github.com/user-attachments/assets/976964ce-0459-4744-bc01-1ac32a947987


