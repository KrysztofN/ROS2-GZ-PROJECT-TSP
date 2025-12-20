import rclpy
from rclpy.node import Node
import subprocess
import sys
import json
import random

class ObstacleChanger(Node):
    def __init__(self, maze_name, marker_positions, cone_positions):
        super().__init__('obstacle_changer')
        
        self.maze_name = maze_name
        self.marker_positions = marker_positions
        self.cone_positions = cone_positions
        self.marker_states = [True] * len(marker_positions) 
        self.cone_states = [True] * len(cone_positions) 
        
        # Timers for markers
        for i in range(len(marker_positions)):
            self.create_timer(12.0, lambda idx=i: self.toggle_marker(idx))
        
        # Timers for cones
        for i in range(len(cone_positions)):
            self.create_timer(12.0, lambda idx=i: self.toggle_cone(idx))
        

        self.get_logger().info(f'Traffic lights and cones initialized!')
    
    def toggle_marker(self, marker_idx):
        x, y = self.marker_positions[marker_idx]
        marker_id = marker_idx + 1
        
        is_red = self.marker_states[marker_idx]
        
        if is_red:
            self.remove_marker(f"stop_marker_{marker_id}_red")
            self.spawn_marker(x, y, f"stop_marker_{marker_id}_green", 0.0, 1.0, 0.0)
            self.get_logger().info(f'Traffic light {marker_id} changed to GREEN!')
            self.marker_states[marker_idx] = False
        else:
            self.remove_marker(f"stop_marker_{marker_id}_green")
            self.spawn_marker(x, y, f"stop_marker_{marker_id}_red", 1.0, 0.0, 0.0)
            self.get_logger().info(f'Traffic light {marker_id} changed to RED!')
            self.marker_states[marker_idx] = True
    
    def toggle_cone(self, cone_idx):
        x, y = self.cone_positions[cone_idx]
        cone_id = cone_idx + 1
        
        cone_present = self.cone_states[cone_idx]
        
        if cone_present:
            self.remove_marker(f"stop_cone_{cone_id}")
            self.get_logger().info(f'Cone {cone_id} removed!')
            self.cone_states[cone_idx] = False
        else:
            self.spawn_cone(x, y, f"stop_cone_{cone_id}")
            self.get_logger().info(f'Cone {cone_id} spawned!')
            self.cone_states[cone_idx] = True
    
    def remove_marker(self, marker_name):
        cmd = [
            "gz", "service", "-s", f"/world/{self.maze_name}.sdf/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000",
            "--req", f'name: "{marker_name}" type: 2'
        ]
        
        try:
            subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        except Exception as e:
            self.get_logger().error(f'Error removing {marker_name}: {e}')
    
    def spawn_marker(self, x, y, marker_name, r, g, b):
        sdf = f"""<?xml version="1.0"?>
<sdf version="1.8">
    <model name="{marker_name}">
        <static>true</static>
        <pose>{x} {y} 0.01 0 0 0</pose>
        <link name="link">
            <visual name="visual">
                <geometry>
                    <box>
                        <size>2.0 2.0 0.02</size>
                    </box>
                </geometry>
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
        
        try:
            subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        except Exception as e:
            self.get_logger().error(f'Error spawning marker {marker_name}: {e}')
    
    def spawn_cone(self, x, y, cone_name):
        cmd = [
            "gz", "service", "-s", f"/world/{self.maze_name}.sdf/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000",
            "--req", f'sdf_filename: "https://fuel.gazebosim.org/1.0/adlarkin/models/Construction Cone Label Test", name: "{cone_name}", pose: {{position: {{x: {x}, y: {y}, z: 0.01}}}}'
        ]
        
        try:
            subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        except Exception as e:
            self.get_logger().error(f'Error spawning cone {cone_name}: {e}')


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
    free_indices = []
    for i in range(len(occupancy_grid)):
        for j in range(len(occupancy_grid[0])):
            if occupancy_grid[i][j] == 0:
                free_indices.append((j, i))
    return random.sample(free_indices, min(num, len(free_indices)))


def main(args=None):
    maze_name = sys.argv[1]
    num_of_traffic_lights = int(sys.argv[2])
    num_of_cones = int(sys.argv[3]) 
    
    with open('../worlds/config/maze_mapping.json', 'r') as file:
        data = json.load(file)
    
    maze_map = data[maze_name]["occupancy_grid"]
    
    marker_positions_grid = get_traffic_light_stops(maze_map, num_of_traffic_lights)
    marker_positions_world = gridToWorld(marker_positions_grid)
    
    all_positions = get_traffic_light_stops(maze_map, num_of_traffic_lights + num_of_cones)
    cone_positions_grid = all_positions[num_of_traffic_lights:]  
    cone_positions_world = gridToWorld(cone_positions_grid)
    
    rclpy.init(args=args)
    
    print(f"Spawning {len(marker_positions_world)} traffic lights and {len(cone_positions_world)} cones...")
    color_changer = ObstacleChanger(maze_name, marker_positions_world, cone_positions_world)
    
    for i, (x, y) in enumerate(marker_positions_world):
        color_changer.spawn_marker(x, y, f"stop_marker_{i+1}_red", 1.0, 0.0, 0.0)
    
    for i, (x, y) in enumerate(cone_positions_world):
        color_changer.spawn_cone(x, y, f"stop_cone_{i+1}")
    
    try:
        rclpy.spin(color_changer)
    except KeyboardInterrupt:
        print("\nShutting down traffic lights and cones...")
    finally:
        color_changer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()