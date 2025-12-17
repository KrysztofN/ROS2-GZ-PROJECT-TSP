import json
import numpy as np
import random
from collections import defaultdict

def gridToWorldPath(path, width=9, height=9, cell_size=2.0):
    new_path = []
    for coordinates in path:
        # Flip Y axis: grid (0,0) is top-left, world (0,0) is center
        grid_x = coordinates[0]
        grid_y = height - 1 - coordinates[1]  # FLIP Y!
        
        world_x = (grid_x - width / 2) * cell_size
        world_y = (grid_y - height / 2) * cell_size
        
        new_path.append((world_x, world_y))
    return new_path

def gridToWorld(point, width=9, height=9, cell_size=2.0):
    grid_x = point[0]
    grid_y = height - 1 - point[1]
    world_x = (grid_x - width / 2) * cell_size
    world_y = (grid_y - height / 2) * cell_size
        
    return (world_x, world_y)

def compute_next_goal(maze_name):
    
    with open('../worlds/config/maze_mapping.json', 'r') as file:
        data = json.load(file)
    
    maze_data = data[maze_name]
    maze_map = maze_data["occupancy_grid"]
    
    free_indicies = []

    for i in range(len(maze_map)):
        for j in range(len(maze_map[0])):
            if maze_map[i][j] == 0:
                free_indicies.append(((j, i)))
    
    return gridToWorld(random.choice(free_indicies))


def get_tsp_waypoints(occupancy_grid, num=3):
    free_indicies = []
    for i in range(len(occupancy_grid)):
        for j in range(len(occupancy_grid[0])):
            if occupancy_grid[i][j] == 0:
                free_indicies.append(((j, i)))
    return random.sample(free_indicies, num)
