import rclpy
from rclpy.node import Node
import subprocess
import sys
import json
import random

class MarkerColorChanger(Node):
    def __init__(self, maze_name, marker_positions):
        super().__init__('marker_color_changer')
        
        self.maze_name = maze_name
        self.marker_positions = marker_positions
        self.marker_states = [True] * len(marker_positions)
        
        for i in range(len(marker_positions)):
            self.create_timer(12.0, lambda idx=i: self.toggle_marker(idx))
        
        self.get_logger().info(f'Traffic light initialized!')
    
    def toggle_marker(self, marker_idx):
        x, y = self.marker_positions[marker_idx]
        marker_id = marker_idx + 1
        
        self.marker_states[marker_idx] = not self.marker_states[marker_idx]
        
        if self.marker_states[marker_idx]:
            r, g, b = 1.0, 0.0, 0.0  
            color = "RED"
        else:
            r, g, b = 0.0, 1.0, 0.0 
            color = "GREEN"
        
        self.get_logger().info(f'Traffic light {marker_id} changed to {color}!')
        
        # Removing and respawning 
        self.remove_marker(marker_id)
        self.spawn_marker(x, y, marker_id, r, g, b)
    
    def remove_marker(self, marker_id):
        cmd = [
            "gz", "service", "-s", f"/world/{self.maze_name}.sdf/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000",
            "--req", f'name: "stop_marker_{marker_id}" type: 2'
        ]
        subprocess.run(cmd, capture_output=True, text=True, timeout=5)
    
    def spawn_marker(self, x, y, marker_id, r, g, b):
        sdf = f"""<?xml version="1.0"?>
<sdf version="1.8">
    <model name="stop_marker_{marker_id}">
        <static>true</static>
        <pose>{x} {y} 0.01 0 0 0</pose>
        <link name="link">
            <visual name="visual">
                <geometry><box><size>2.0 2.0 0.02</size></box></geometry>
                <material>
                    <ambient>{r} {g} {b} 1</ambient>
                    <diffuse>{r} {g} {b} 1</diffuse>
                </material>
            </visual>
        </link>
    </model>
</sdf>"""
        
        sdf_escaped = sdf.replace('"', '\\"').replace('\n', ' ')
        cmd = [
            "gz", "service", "-s", f"/world/{self.maze_name}.sdf/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000",
            "--req", f'sdf: "{sdf_escaped}"'
        ]
        subprocess.run(cmd, capture_output=True, text=True, timeout=5)


def gridToWorld(path, width=9, height=9, cell_size=2.0):
    new_path = []
    for coordinates in path:
        grid_x = coordinates[0]
        grid_y = height - 1 - coordinates[1]
        world_x = (grid_x - width / 2) * cell_size
        world_y = (grid_y - height / 2) * cell_size
        new_path.append((world_x, world_y))
    return new_path


def get_traffic_light_stops(occupancy_grid, num=3):
    return [(1, 3)]
    free_indices = []
    for i in range(len(occupancy_grid)):
        for j in range(len(occupancy_grid[0])):
            if occupancy_grid[i][j] == 0:
                free_indices.append((j, i))
    return random.sample(free_indices, num)


def main(args=None):
    maze_name = sys.argv[1]
    num_of_traffic_lights = int(sys.argv[2])
    
    with open('../worlds/config/maze_mapping.json', 'r') as file:
        data = json.load(file)
    
    maze_map = data[maze_name]["occupancy_grid"]
    positions = get_traffic_light_stops(maze_map, num_of_traffic_lights)
    world_positions = gridToWorld(positions)
    
    rclpy.init(args=args)
    
    print("Red lights...")
    color_changer = MarkerColorChanger(maze_name, world_positions)
    for i, (x, y) in enumerate(world_positions):
        color_changer.spawn_marker(x, y, i+1, 1.0, 0.0, 0.0)
    
    try:
        rclpy.spin(color_changer)
    except KeyboardInterrupt:
        pass
    finally:
        color_changer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()