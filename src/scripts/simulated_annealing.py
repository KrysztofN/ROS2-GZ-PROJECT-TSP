import numpy as np
import random
import math
from typing import List, Tuple, Optional
import json
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from a_star_path import a_star_search, is_valid, is_unblocked  
import os


class OccupancyGridTSP:
    def __init__(self, 
                 occupancy_grid,
                 points_to_visit,
                 start_point,
                 finish_point = None,
                 cell_size = 2.0):
    
        self.grid = occupancy_grid
        self.cell_size = cell_size
        self.start_point = start_point
        self.finish_point = finish_point

        
        import a_star_path
        a_star_path.ROW = len(self.grid)
        a_star_path.COL = len(self.grid[0])
        
        # All points: [start, intermediate points, finish]
        self.all_points = [start_point] + points_to_visit + [finish_point]
        self.finish_idx = len(self.all_points) - 1
        
        self.n_points = len(self.all_points)
        self.depot_idx = 0  
        
        # Distance matrix
        self.dist_matrix = self._compute_distance_matrix()

    
    def _calculate_min_fuel(self, distance_matrix):
        start_idx = 0
        end_idx = len(distance_matrix) - 1
        
        min_fuel = distance_matrix[start_idx, end_idx]
        
        for i in range(1, len(distance_matrix) - 1): 
            dist_from_start = distance_matrix[start_idx, i]
            dist_from_end = distance_matrix[end_idx, i]
            
            min_dist_to_waypoint = min(2 * dist_from_start, 2 * dist_from_end)
            min_fuel = max(min_fuel, min_dist_to_waypoint)
        
        return min_fuel

    def _astar_distance(self, start, goal):
        
        if start == goal:
            return 0.0
        
        src = [start[1], start[0]]  
        dest = [goal[1], goal[0]]   
        path = a_star_search(self.grid, src, dest)
        
        if not path or len(path) == 0:
            return None
        
        path_length_cells = len(path) - 1        
        return path_length_cells * self.cell_size
    
    def _compute_distance_matrix(self):
        n = len(self.all_points)
        dist_matrix = np.full((n, n), float('inf'))
        
        for i in range(n):
            for j in range(n):
                if i == j:
                    dist_matrix[i, j] = 0
                else:
                    distance = self._astar_distance(self.all_points[i], self.all_points[j])
                    if distance is not None:
                        dist_matrix[i, j] = distance
        
        return dist_matrix
    
    def calculate_route_cost(self, route):
        
        total_distance = 0
        current_pos = self.depot_idx
        
        for next_pos in route:
            distance = self.dist_matrix[current_pos, next_pos]
            
            if distance == float('inf'):
                return float('inf'), False
            
            total_distance += distance
            current_pos = next_pos
        
        final_distance = self.dist_matrix[current_pos, self.finish_idx]
        if final_distance == float('inf'):
            return float('inf'), False
        
        total_distance += final_distance
        return total_distance, True
    
    def generate_neighbor(self, route: List[int]):
        if len(route) < 2:
            return route.copy()
        
        new_route = route.copy()
        operation = random.choice(['swap', 'reverse', 'insert'])
        
        if operation == 'swap':
            i, j = random.sample(range(len(new_route)), 2)
            new_route[i], new_route[j] = new_route[j], new_route[i]
        
        elif operation == 'reverse':
            if len(new_route) >= 2:
                i, j = sorted(random.sample(range(len(new_route)), 2))
                new_route[i:j+1] = list(reversed(new_route[i:j+1]))
        
        elif operation == 'insert':
            i = random.randint(0, len(new_route) - 1)
            point = new_route.pop(i)
            j = random.randint(0, len(new_route))
            new_route.insert(j, point)
        
        return new_route
    
    def simulated_annealing(self,
                           initial_temp: float = 1000,
                           cooling_rate: float = 0.995,
                           min_temp: float = 1,
                           max_iterations: int = 10000,
                           verbose: bool = True) -> Tuple[List[int], float]:

        cities_to_visit = list(range(1, self.n_points - 1))
        
        
        if not cities_to_visit:
            return [], 0.0
        
        current_route = self._nearest_neighbor_initial(cities_to_visit)
        current_cost, is_valid = self.calculate_route_cost(current_route)
        
        if not is_valid:
            for _ in range(10):
                random.shuffle(cities_to_visit)
                current_route = cities_to_visit.copy()
                current_cost, is_valid = self.calculate_route_cost(current_route)
                if is_valid:
                    break
        
        if not is_valid:
            return current_route, float('inf')
        
        best_route = current_route.copy()
        best_cost = current_cost
        
        temperature = initial_temp
        iteration = 0
        accepted_moves = 0
        
        while temperature > min_temp and iteration < max_iterations:
            new_route = self.generate_neighbor(current_route)
            new_cost, is_valid = self.calculate_route_cost(new_route)
            
            if not is_valid:
                new_cost = float('inf')
            
            if new_cost < current_cost:
                current_route = new_route
                current_cost = new_cost
                accepted_moves += 1
                
                if current_cost < best_cost:
                    best_route = current_route.copy()
                    best_cost = current_cost
                        
            
            elif new_cost != float('inf'):
                delta = new_cost - current_cost
                acceptance_prob = math.exp(-delta / temperature)
                
                if random.random() < acceptance_prob:
                    current_route = new_route
                    current_cost = new_cost
                    accepted_moves += 1
            
            temperature *= cooling_rate
            iteration += 1
            
            if verbose and iteration % 1000 == 0:
                acceptance_rate = accepted_moves / 1000
                accepted_moves = 0
        
        return best_route, best_cost
    
    def held_karp(self):
        cities_to_visit = list(range(1, self.n_points - 1))
        n = len(cities_to_visit)
        
        if n == 0:
            return [], 0.0
        
        direct_cost = self.dist_matrix[self.depot_idx, self.finish_idx]
        if direct_cost == float('inf'):
            return [], float('inf')
        
        if n == 1:
            city = cities_to_visit[0]
            cost = (self.dist_matrix[self.depot_idx, city] + 
                    self.dist_matrix[city, self.finish_idx])
            if cost == float('inf'):
                return [], float('inf')
            return [city], cost
        
        bit_to_city = {i: cities_to_visit[i] for i in range(n)}
        
        C = {}
        
        for k in range(n):
            city = bit_to_city[k]
            cost = self.dist_matrix[self.depot_idx, city]
            C[(1 << k, k)] = (cost, -1) 
        
        from itertools import combinations
        for subset_size in range(2, n + 1):
            for subset in combinations(range(n), subset_size):
                bits = 0
                for bit in subset:
                    bits |= 1 << bit
                
                for k in subset:
                    prev_bits = bits & ~(1 << k)
                    
                    min_cost = float('inf')
                    best_prev = None
                    
                    for m in subset:
                        if m == k:
                            continue
                        
                        if (prev_bits, m) not in C:
                            continue
                        
                        prev_cost, _ = C[(prev_bits, m)]
                        edge_cost = self.dist_matrix[bit_to_city[m], bit_to_city[k]]
                        
                        if prev_cost == float('inf') or edge_cost == float('inf'):
                            continue
                        
                        total = prev_cost + edge_cost
                        if total < min_cost:
                            min_cost = total
                            best_prev = m
                    
                    C[(bits, k)] = (min_cost, best_prev)
        
        all_bits = (1 << n) - 1
        min_total = float('inf')
        best_last = None
        
        for k in range(n):
            if (all_bits, k) not in C:
                continue
            
            path_cost, _ = C[(all_bits, k)]
            final_edge = self.dist_matrix[bit_to_city[k], self.finish_idx]
            
            if path_cost == float('inf') or final_edge == float('inf'):
                continue
            
            total = path_cost + final_edge
            if total < min_total:
                min_total = total
                best_last = k
        
        if best_last is None:
            return [], float('inf')
        
        path = []
        bits = all_bits
        current = best_last
        
        while current != -1:
            path.append(bit_to_city[current])
            if bits == (1 << current):
                break
            
            new_bits = bits & ~(1 << current)
            _, prev = C[(bits, current)]
            bits = new_bits
            current = prev
        
        path.reverse()
        
        return path, min_total
    


    def _nearest_neighbor_initial(self, cities: List[int]):
        route = []
        unvisited = set(cities)
        current = self.depot_idx
        
        while unvisited:
            nearest = min(unvisited, 
                         key=lambda x: self.dist_matrix[current, x])
            route.append(nearest)
            unvisited.remove(nearest)
            current = nearest
        
        return route
    
    def get_full_path(self, route: List[int]) -> List[Tuple[int, int]]:
        full_path = []
        route_indices = [0] + route + [self.finish_idx]
        
        for i in range(len(route_indices) - 1):
            from_point = self.all_points[route_indices[i]]
            to_point = self.all_points[route_indices[i + 1]]
            
            # Get A* path between two points
            src = [from_point[1], from_point[0]]  
            dest = [to_point[1], to_point[0]]
            
            segment_path = a_star_search(self.grid, src, dest)
            
            start_idx = 0 if i == 0 else 1
            for coord in segment_path[start_idx:]:
                full_path.append((coord[1], coord[0]))  
        
        return full_path
    

    def visualize_tsp(self, path, waypoints, occupancy_grid, maze_name, cell_size=2.0):
        matplotlib.use('agg')
        height = len(occupancy_grid)
        width = len(occupancy_grid[0])
        
        fig, ax = plt.subplots(figsize=(10, 10))
        
        for i in range(height):
            for j in range(width):
                if occupancy_grid[i][j] == 1:  
                    rect = plt.Rectangle((j, height - i - 1), 1, 1, 
                                        facecolor='black', edgecolor='black', linewidth=0.5)
                    ax.add_patch(rect)
        
        x_coords = []
        y_coords = []
        for (x, y) in path:
            x_coords.append(x + 1/cell_size)
            y_coords.append(height - (y + 1/cell_size))

        x_waypoint_coords = []
        y_waypoint_coords = []
        for (x, y) in waypoints:
            x_waypoint_coords.append(x + 1/cell_size)
            y_waypoint_coords.append(height - (y + 1/cell_size))
        
        plt.plot(x_coords, y_coords, 'y-', linewidth=2, alpha=0.7, label='Path')
        
        plt.scatter(x_coords[1:-1], y_coords[1:-1], c='blue', s=50, zorder=3, label='Full path')
        plt.scatter(x_waypoint_coords, y_waypoint_coords, c='gold', s=300, zorder=4, label='Waypoints')
        plt.scatter(x_coords[0], y_coords[0], c='green', s=400, zorder=5, label='Start')
        plt.scatter(x_coords[-1], y_coords[-1], c='red', s=400, zorder=5, label='End')

        x_axis = []
        y_axis = []
        for i in range(1, len(x_coords)):
            x_prev = x_coords[i-1]
            y_prev = y_coords[i-1]
            x_curr = x_coords[i]
            y_curr = y_coords[i]
            x_axis.extend(np.linspace(x_prev, x_curr, 5))
            y_axis.extend(np.linspace(y_prev, y_curr, 5))

        animated_path, = ax.plot([], [], 'cyan', linewidth=3, label='Current Path', zorder=6)
        current_point, = ax.plot([], [], 'mo', markersize=15, label='Current Position', zorder=7)
        
        ax.set_xlim(0, width)
        ax.set_ylim(0, height)
        ax.set_xticks(np.arange(0, width + 1/cell_size, 1/cell_size))
        ax.set_yticks(np.arange(0, height + 1/cell_size, 1/cell_size))
        
        ax.grid(True, alpha=0.3, linewidth=0.5)
        ax.set_aspect('equal', adjustable='box')
        ax.legend(loc="upper right", numpoints=1, fontsize=10, markerscale=0.6)
        ax.set_title('TSP Path in Maze')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')

        def update_frame(frame):
            animated_path.set_data(x_axis[:frame+1], y_axis[:frame+1])
            current_point.set_data([x_axis[frame]], [y_axis[frame]])
            return animated_path, current_point
        
        anim = animation.FuncAnimation(fig, update_frame,
                                       frames=len(x_axis),
                                       interval=20,
                                       blit=True,
                                       repeat=True)


        plt.tight_layout()
        anim.save(filename=f"../tsp_animations/{maze_name}.mp4", writer="ffmpeg")
        plt.close()       


def solve_tsp(maze_name, start, finish, occupancy_grid, points_to_visit):
    
    # SP solver
    tsp = OccupancyGridTSP(
        occupancy_grid=occupancy_grid,
        points_to_visit=points_to_visit,
        start_point=start,
        finish_point=finish,  
        cell_size=2.0
    )
    
    # Solve Simulated Annealing
    best_route_sa, best_cost_sa = tsp.simulated_annealing(
        initial_temp=1000,
        cooling_rate=0.995,
        min_temp=0.1,
        max_iterations=10000,
        verbose=True
    )

    best_route_hk, best_cost_hk = tsp.held_karp()

    tsp_result = {}
    os.makedirs('results', exist_ok = True)
    
    if best_cost_sa != float('inf') and best_cost_hk != float('inf'):
        print(f"TSP cost: {best_cost_sa:.2f}m")
        
        full_path_sa = tsp.get_full_path(best_route_sa)
        full_path_hk = tsp.get_full_path(best_route_hk)

        tsp_result['Simulated annealing'] = {"route": best_route_sa, "cost" : best_cost_sa}
        tsp_result['Held-Karp'] = {"route": best_route_hk, "cost" : best_cost_hk}

        tsp_result['Simulated annealing']['full route'] = [list(coord) for coord in full_path_sa]
        tsp_result['Held-Karp']['full route'] = [list(coord) for coord in full_path_hk]


        with open('results/tsp_results.json', 'w') as f:
            json.dump(tsp_result, f, indent=2)
        
        print("Saving tsp animation...can take a while")
        try:
            
            tsp.visualize_tsp([start] + full_path_sa, points_to_visit, occupancy_grid, maze_name + "_sa")
            tsp.visualize_tsp([start] + full_path_hk, points_to_visit, occupancy_grid, maze_name + "_hk")
            print("Animations saved!")
        except:
            print("Animation could not be saved!")
    return full_path_sa