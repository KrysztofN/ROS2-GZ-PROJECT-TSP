# This code is build upon maze generation algorithm in python
# contribution: https://inventwithpython.com/recursion/chapter11.html
# Code aims to create sdf worlds with unique mazes with start and finish point. 
# In result we have complete sdf worlds as well as configuration file
# with maze meta-data, occupancy grid and start/finish coordinates

import random
import json
import os
import time
import sys
from pathlib import Path
import os

# Maze generation parameters
WIDTH = 9  # Width of the maze 
HEIGHT = 9  # Height of the maze 
CELL_SIZE = 2.0  # Size of each cell in meters
WALL_HEIGHT = 1.0  # Height of walls in meters
WORLDS_DIR = Path("../worlds/mazes")
CONFIG_DIR = Path("../worlds/config")
SEED = int(time.time())
random.seed(SEED)

NUM_WALLS_TO_REMOVE = 5

assert WIDTH % 2 == 1 and WIDTH >= 3
assert HEIGHT % 2 == 1 and HEIGHT >= 3


# Maze characters
EMPTY = ' '
MARK = '@'
WALL = chr(9608)
NORTH, SOUTH, EAST, WEST = 'n', 's', 'e', 'w'

def create_maze_structure():
    maze = {}
    for x in range(WIDTH):
        for y in range(HEIGHT):
            maze[(x, y)] = WALL
    return maze

def printMaze(maze, markX=None, markY=None):
    for y in range(HEIGHT):
        for x in range(WIDTH):
            if markX == x and markY == y:
                print(MARK, end='')
            else:
                print(maze[(x, y)], end='')
        print()

def visit(maze, x, y):
    maze[(x, y)] = EMPTY
    
    while True:
        unvisitedNeighbors = []
        if y > 1 and (x, y - 2) not in hasVisited:
            unvisitedNeighbors.append(NORTH)
        if y < HEIGHT - 2 and (x, y + 2) not in hasVisited:
            unvisitedNeighbors.append(SOUTH)
        if x > 1 and (x - 2, y) not in hasVisited:
            unvisitedNeighbors.append(WEST)
        if x < WIDTH - 2 and (x + 2, y) not in hasVisited:
            unvisitedNeighbors.append(EAST)
        
        if len(unvisitedNeighbors) == 0:
            return
        else:
            nextIntersection = random.choice(unvisitedNeighbors)
            
            if nextIntersection == NORTH:
                nextX, nextY = x, y - 2
                maze[(x, y - 1)] = EMPTY
            elif nextIntersection == SOUTH:
                nextX, nextY = x, y + 2
                maze[(x, y + 1)] = EMPTY
            elif nextIntersection == WEST:
                nextX, nextY = x - 2, y
                maze[(x - 1, y)] = EMPTY
            elif nextIntersection == EAST:
                nextX, nextY = x + 2, y
                maze[(x + 1, y)] = EMPTY
            
            hasVisited.append((nextX, nextY))
            visit(maze, nextX, nextY)

def add_multiple_paths(maze, num_walls_to_remove=5):
    
    removable_walls = []
    
    for x in range(1, WIDTH - 1):
        for y in range(1, HEIGHT - 1):
            if maze[(x, y)] == WALL:
                # Horizontal wall (between left and right cells)
                if (x % 2 == 0 and y % 2 == 1 and 
                    maze.get((x-1, y)) == EMPTY and 
                    maze.get((x+1, y)) == EMPTY):
                    removable_walls.append((x, y))
                # Vertical wall (between top and bottom cells)
                elif (x % 2 == 1 and y % 2 == 0 and 
                      maze.get((x, y-1)) == EMPTY and 
                      maze.get((x, y+1)) == EMPTY):
                    removable_walls.append((x, y))
    
    walls_to_remove = min(num_walls_to_remove, len(removable_walls))
    if walls_to_remove > 0:
        walls_removed = random.sample(removable_walls, walls_to_remove)
        
        for x, y in walls_removed:
            maze[(x, y)] = EMPTY
        
        return len(walls_removed)
    
    return 0

def getEmptySpaces():
    empty_spaces = []
    for x in range(WIDTH):
        for y in range(HEIGHT):
            if maze[(x, y)] == EMPTY: 
                empty_spaces.append((x, y))
    return empty_spaces

def selectStartEnd():
    empty_spaces = getEmptySpaces()
    if len(empty_spaces) < 2:
        raise ValueError("Not enough empty spaces for start and end points")
    
    # Selecting two random different points
    start = random.choice(empty_spaces)
    empty_spaces.remove(start)
    end = random.choice(empty_spaces)
    
    return start, end

def gridToWorld(grid_x, grid_y):
    world_x = (grid_x - WIDTH / 2) * CELL_SIZE
    world_y = (grid_y - HEIGHT / 2) * CELL_SIZE
    return world_x, world_y

def createOccupancyGrid():
    grid = []
    for y in range(HEIGHT -1, -1, -1):
        row = []
        for x in range(WIDTH):
            row.append(1 if maze[(x, y)] == WALL else 0)
        grid.append(row)
    return grid

def generateSDF(start, end, filename="maze_world.sdf"):
    
    sdf_content = f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="{filename}">
    
    <!-- Physics -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    
    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>200 200</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>200 200</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
'''
    
    # Generating walls
    wall_id = 0
    for x in range(WIDTH):
        for y in range(HEIGHT):
            if maze[(x, y)] == WALL:
                world_x, world_y = gridToWorld(x, y)
                sdf_content += f'''    <!-- Wall at grid ({x}, {y}) -->
    <model name="wall_{wall_id}">
      <static>true</static>
      <pose>{world_x} {world_y} {WALL_HEIGHT/2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{CELL_SIZE} {CELL_SIZE} {WALL_HEIGHT}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{CELL_SIZE} {CELL_SIZE} {WALL_HEIGHT}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.4 1</ambient>
            <diffuse>0.4 0.4 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
'''
                wall_id += 1
    
    # White marker for start point
    start_x, start_y = gridToWorld(start[0], start[1])
    sdf_content += f'''    <!-- Start Point (Green) -->
    <model name="start_marker">
      <static>true</static>
      <pose>{start_x} {start_y} 0.001 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>0.002</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <emissive>0 0 0 1</emissive>
          </material>
        </visual>
      </link>
    </model>
    
'''
    
    # Black marker for end point
    end_x, end_y = gridToWorld(end[0], end[1])
    sdf_content += f'''    <!-- End Point (Red) -->
    <model name="end_marker">
      <static>true</static>
      <pose>{end_x} {end_y} 0.001 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>0.002</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
            <emissive>0 0 0 1</emissive>
          </material>
        </visual>
      </link>
    </model>'''
    # Calculating robot spawn position at start point
    robot_x, robot_y = gridToWorld(start[0], start[1])
    
    sdf_content += f'''    
    <!-- Robot spawned at start point -->
    <include>
      <uri>model://maze_robot</uri>
      <name>robot</name>
      <pose>{robot_x} {robot_y} 0.151 0 0 0</pose>
    </include>
    
  </world>
</sdf>'''
    
    return sdf_content

def generateMappingJSON(occupancy_grid, start, end, num, filename="maze_mapping.json"):
    
    start_world = gridToWorld(start[0], start[1])
    end_world = gridToWorld(end[0], end[1])
    
    start_grid_y_flipped = HEIGHT - 1 - start[1]
    end_grid_y_flipped = HEIGHT - 1 - end[1]
    
    maze_data = {
        "maze_info": {
            "width": WIDTH,
            "height": HEIGHT,
            "cell_size": CELL_SIZE,
            "wall_height": WALL_HEIGHT,
            "seed": SEED
        },
        "occupancy_grid": occupancy_grid,
        "start_point": {
            "grid": {"x": start[0], "y": start_grid_y_flipped},  
            "world": {"x": start_world[0], "y": start_world[1]}
        },
        "end_point": {
            "grid": {"x": end[0], "y": end_grid_y_flipped}, 
            "world": {"x": end_world[0], "y": end_world[1]}
        }
    }
    
    return maze_data

def batch(i, json_content):
    global hasVisited, maze
    print(f"Generating maze {i}")
    
    maze = create_maze_structure()
    hasVisited = [(1, 1)]

    random.seed(time.time() * 1000 + i)
    visit(maze, 1, 1)
    
    # Adding multiple solution paths
    num_removed = add_multiple_paths(maze, num_walls_to_remove=NUM_WALLS_TO_REMOVE)
    
    start, end = selectStartEnd()
    
    occupancy_grid = createOccupancyGrid()
    
    WORLDS_DIR.mkdir(parents=True, exist_ok=True)
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    
    sdf_filename = f"maze_{i}.sdf"
    
    sdf_content = generateSDF(start, end, sdf_filename)
    maze_data = generateMappingJSON(occupancy_grid, start, end, i)
    all_mazes_data[f"maze_{i}"] = maze_data

    with open(os.path.join(WORLDS_DIR,sdf_filename), 'w') as f:
        f.write(sdf_content)
    

if __name__ == "__main__":
    num_of_mazes = int(sys.argv[1])
    all_mazes_data = {}
    json_filename = "maze_mapping.json"

    for i in range(num_of_mazes):
        batch(i+1, all_mazes_data)
    
    with open(os.path.join(CONFIG_DIR, json_filename), 'w') as f:
        json.dump(all_mazes_data, f, indent=2)
    
    print("\nGeneration complete!")
    print(f"Generated {num_of_mazes} mazes with {NUM_WALLS_TO_REMOVE} alternative paths each")