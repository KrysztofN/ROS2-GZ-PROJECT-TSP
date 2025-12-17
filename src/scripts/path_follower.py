import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from travelling_salesman import solve_tsp
from useful_functions import get_tsp_waypoints
import sys
import math
import json

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/bottom', self.camera_callback, 10)
        
        self.bridge = CvBridge()
        self.is_stopped_at_red = False
        
        maze_name = sys.argv[1]
        with open('../worlds/config/maze_mapping.json', 'r') as file:
            data = json.load(file)
        
        self.expected_start_x = data[maze_name]["start_point"]["world"]["x"]
        self.expected_start_y = data[maze_name]["start_point"]["world"]["y"]
        
        start_x_grid = data[maze_name]["start_point"]["grid"]["x"]
        start_y_grid = data[maze_name]["start_point"]["grid"]["y"]
        end_x_grid = data[maze_name]["end_point"]["grid"]["x"]
        end_y_grid = data[maze_name]["end_point"]["grid"]["y"]
        start_point = (start_x_grid, start_y_grid)
        end_point = (end_x_grid, end_y_grid)
        occupancy_grid = data[maze_name]["occupancy_grid"]
        points_to_visit = get_tsp_waypoints(occupancy_grid)

        tsp_route = solve_tsp(maze_name, start_point, end_point, occupancy_grid, points_to_visit)
        if not tsp_route:
            print("\nExiting...")
            rclpy.shutdown()
            sys.exit(0)
        self.waypoints_original = self.gridToWorld(tsp_route)

        self.waypoints = []
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.current_waypoint_idx = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        
        self.max_linear_speed = 0.5  
        self.max_angular_speed = 0.5  
        self.position_tolerance = 0.08  
        self.angle_tolerance = 0.08  
        self.kp_linear = 1.0
        self.kp_angular = 2.0
        self.state = 'ROTATE'
        self.log_counter = 0
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Waypoint Navigator initialized')
    
    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            lower_green = np.array([40, 100, 100])
            upper_green = np.array([80, 255, 255])
            
            mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            mask_green = cv2.inRange(hsv, lower_green, upper_green)
            
            red_pixels = cv2.countNonZero(mask_red)
            green_pixels = cv2.countNonZero(mask_green)
            threshold = 500
            
            if red_pixels > threshold:
                if not self.is_stopped_at_red:
                    self.get_logger().info('RED DETECTED! Stopping...')
                self.is_stopped_at_red = True
            elif green_pixels > threshold and self.is_stopped_at_red:
                self.get_logger().info('GREEN DETECTED! Resuming...')
                self.is_stopped_at_red = False
                
        except Exception as e:
            self.get_logger().error(f'Camera error: {e}')
    
    def gridToWorld(self, path, width=9, height=9, cell_size=2.0):
        new_path = []
        for coordinates in path:
            grid_x = coordinates[0]
            grid_y = height - 1 - coordinates[1]
            world_x = (grid_x - width / 2) * cell_size
            world_y = (grid_y - height / 2) * cell_size
            new_path.append((world_x, world_y))
        return new_path
    
    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        
        if not self.odom_received:
            self.odom_received = True
            self.offset_x = self.current_x - self.expected_start_x
            self.offset_y = self.current_y - self.expected_start_y
            
            self.waypoints = []
            for wx, wy in self.waypoints_original:
                self.waypoints.append((wx + self.offset_x, wy + self.offset_y))
            self.waypoints = self.waypoints[1:]
            
            self.get_logger().info(f'Offset: ({self.offset_x:.2f}, {self.offset_y:.2f})')
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def get_target_angle(self, target_x, target_y):
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        return math.atan2(dy, dx)
    
    def get_distance_to_target(self, target_x, target_y):
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        return math.sqrt(dx**2 + dy**2)
    
    def control_loop(self):
        if not self.odom_received:
            return
        
        if self.is_stopped_at_red:
            self.stop_robot()
            return
        
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints reached!')
            self.stop_robot()
            self.timer.cancel()
            return
        
        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        distance = self.get_distance_to_target(target_x, target_y)
        target_angle = self.get_target_angle(target_x, target_y)
        angle_error = self.normalize_angle(target_angle - self.current_yaw)
        
        twist = Twist()
        should_log = (self.log_counter % 10 == 0)
        self.log_counter += 1
        
        if self.state == 'ROTATE':
            if abs(angle_error) > self.angle_tolerance:
                angular_vel = self.kp_angular * angle_error
                twist.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_vel))
                
                if should_log:
                    self.get_logger().info(f'[WP {self.current_waypoint_idx + 1}] ROTATING | Angle: {math.degrees(angle_error):.1f}°')
            else:
                self.state = 'MOVE'
                self.get_logger().info(f'[WP {self.current_waypoint_idx + 1}] Moving forward')
        
        elif self.state == 'MOVE':
            if distance > self.position_tolerance:
                if abs(angle_error) > self.angle_tolerance * 3:
                    self.state = 'ROTATE'
                else:
                    linear_vel = max(0.2, min(self.max_linear_speed, self.kp_linear * distance))
                    angular_vel = self.kp_angular * angle_error * 0.3
                    angular_vel = max(-self.max_angular_speed * 0.5, min(self.max_angular_speed * 0.5, angular_vel))
                    
                    twist.linear.x = linear_vel
                    twist.angular.z = angular_vel
                    
                    if should_log:
                        self.get_logger().info(f'[WP {self.current_waypoint_idx + 1}] MOVING | Distance: {distance:.2f}m')
            else:
                self.get_logger().info(f'[WP {self.current_waypoint_idx + 1}] REACHED')
                self.current_waypoint_idx += 1
                self.state = 'ROTATE'
        
        self.cmd_vel_pub.publish(twist)
    
    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Stopping...')
    finally:
        navigator.stop_robot()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()