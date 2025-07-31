import pygame
import heapq
import math
import sys
import random
import time
from collections import deque

'''
Note:
    - In this code some functions take in a position which is then separated into row and column while
        others take in a position as a tuple (row, col) directly.
    - The grid being used in this code is an odd-q vertical layout hexagonal grid where
        the even columns are shifted up and the odd columns are shifted down.
    - 0 is a walkable cell, 1 is an obstacle.

'''

class HexGrid:
    # Directions: 0=N, 1=NE, 2=SE, 3=S, 4=SW, 5=NW
    EVEN_Q_DIRS = [
        (0, -1),   # N
        (1, -1),   # NE
        (1, 0),    # SE
        (0, 1),    # S
        (-1, 0),   # SW
        (-1, -1)   # NW
    ]
    ODD_Q_DIRS = [
        (0, -1),   # N
        (1, 0),    # NE
        (1, 1),    # SE
        (0, 1),    # S
        (-1, 1),   # SW
        (-1, 0)    # NW
    ]

    def __init__(self, grid):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if grid else 0

    def is_valid(self, row, col):
        return (0 <= row < self.rows and 0 <= col < self.cols and self.grid[row][col] != 1)

    def get_directions(self, col):
        return HexGrid.EVEN_Q_DIRS if col % 2 == 0 else HexGrid.ODD_Q_DIRS

    def move(self, pos, direction):
        row, col = pos
        dir_row, dir_col = self.get_directions(col)[direction]
        return (row + dir_row, col + dir_col)

    def get_neighbors(self, pos):
        row, col = pos
        dirs = self.get_directions(col)
        neighbors = []
        for d, (dr, dc) in enumerate(dirs):
            nr, nc = row + dr, col + dc
            if self.is_valid(nr, nc):
                neighbors.append((nr, nc))
        return neighbors

    def odd_q_to_cube(self, a):
        row, col = a
        x = col
        z = row - (col - (col & 1)) // 2
        y = -x - z
        return (x, y, z)

    def cube_distance(self, a, b):
        a_cube = self.odd_q_to_cube(a)
        b_cube = self.odd_q_to_cube(b)
        return (abs(a_cube[0] - b_cube[0]) + abs(a_cube[1] - b_cube[1]) + abs(a_cube[2] - b_cube[2])) // 2

    def direction_from_to(self, a, b):
        # Returns the direction index from a to b, or None if not adjacent
        row, col = a
        dirs = self.get_directions(col)
        for i, (dr, dc) in enumerate(dirs):
            if (row + dr, col + dc) == b:
                return i
        return None

    def prune_neighbors(self, current, parent):
        """Prune neighbors for JPS - prefer straight line movement."""
        if parent is None:
            # No parent, return all neighbors
            return [i for i in range(6)]
        
        # Get the direction from parent to current
        direction = self.direction_from_to(parent, current)
        if direction is None:
            return [i for i in range(6)]
        
        # For JPS, we prefer to continue in the same direction
        # This is the core JPS concept: prefer straight-line movement
        pruned = [direction]
        
        # Only add other directions if the preferred direction is blocked
        preferred_pos = self.move(current, direction)
        if not self.is_valid(*preferred_pos):
            # If preferred direction is blocked, add all valid neighbors
            for d in range(6):
                if d != direction:
                    neighbor_pos = self.move(current, d)
                    if self.is_valid(*neighbor_pos):
                        pruned.append(d)
        
        return pruned

    def find_path_bfs(self, start, goal):
        """Breadth-First Search pathfinding."""
        if not self.is_valid(*start) or not self.is_valid(*goal):
            return []
        if start == goal:
            return [start]
        
        queue = deque([(start, [start])])
        visited = {start}
        
        while queue:
            current, path = queue.popleft()
            
            if current == goal:
                return path
            
            for neighbor in self.get_neighbors(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        
        return []

    def find_path_dijkstra(self, start, goal):
        """Dijkstra's algorithm pathfinding."""
        if not self.is_valid(*start) or not self.is_valid(*goal):
            return []
        if start == goal:
            return [start]
        
        # Priority queue: (distance, current_pos, path)
        pq = [(0, start, [start])]
        visited = set()
        
        while pq:
            distance, current, path = heapq.heappop(pq)
            
            if current in visited:
                continue
            
            visited.add(current)
            
            if current == goal:
                return path
            
            for neighbor in self.get_neighbors(current):
                if neighbor not in visited:
                    # Cost is 1 for each step in hex grid
                    new_distance = distance + 1
                    heapq.heappush(pq, (new_distance, neighbor, path + [neighbor]))
        
        return []

    def find_path_astar(self, start, goal):
        """A* algorithm pathfinding."""
        if not self.is_valid(*start) or not self.is_valid(*goal):
            return []
        if start == goal:
            return [start]
        
        # Priority queue: (f_score, g_score, current_pos, path)
        pq = [(0 + self.cube_distance(start, goal), 0, start, [start])]
        visited = set()
        g_scores = {start: 0}
        
        while pq:
            f_score, g_score, current, path = heapq.heappop(pq)
            
            if current in visited:
                continue
            
            visited.add(current)
            
            if current == goal:
                return path
            
            for neighbor in self.get_neighbors(current):
                if neighbor not in visited:
                    # Cost is 1 for each step in hex grid
                    new_g_score = g_score + 1
                    
                    if neighbor not in g_scores or new_g_score < g_scores[neighbor]:
                        g_scores[neighbor] = new_g_score
                        h_score = self.cube_distance(neighbor, goal)
                        f_score = new_g_score + h_score
                        heapq.heappush(pq, (f_score, new_g_score, neighbor, path + [neighbor]))
        
        return []

    def find_path_astar_stepwise(self, start, goal, visualization_callback=None):
        """A* algorithm with step-by-step visualization."""
        if not self.is_valid(*start) or not self.is_valid(*goal):
            return []
        if start == goal:
            return [start]
        
        # Priority queue: (f_score, g_score, current_pos, path)
        pq = [(0 + self.cube_distance(start, goal), 0, start, [start])]
        visited = set()
        g_scores = {start: 0}
        frontier = {start}  # Track cells in the frontier
        
        while pq:
            f_score, g_score, current, path = heapq.heappop(pq)
            
            if current in visited:
                continue
            
            visited.add(current)
            frontier.discard(current)  # Remove from frontier when visited
            
            # Call visualization callback if provided
            if visualization_callback:
                visualization_callback(current, visited, frontier, path)
            
            if current == goal:
                return path
            
            for neighbor in self.get_neighbors(current):
                if neighbor not in visited:
                    # Cost is 1 for each step in hex grid
                    new_g_score = g_score + 1
                    
                    if neighbor not in g_scores or new_g_score < g_scores[neighbor]:
                        g_scores[neighbor] = new_g_score
                        h_score = self.cube_distance(neighbor, goal)
                        f_score = new_g_score + h_score
                        heapq.heappush(pq, (f_score, new_g_score, neighbor, path + [neighbor]))
                        frontier.add(neighbor)  # Add to frontier
        
        return []

    def find_path_jps(self, start, goal):
        """Jump Point Search algorithm for hex grids - simplified without jumping."""
        if not self.is_valid(*start) or not self.is_valid(*goal):
            return []
        if start == goal:
            return [start]
        
        open_set = []
        heapq.heappush(open_set, (0 + self.cube_distance(start, goal), 0, start, None))
        came_from = {}
        g_score = {start: 0}
        closed_set = set()
        
        while open_set:
            f, g, current, parent = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            
            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]
            
            # Get pruned neighbors (prefer straight line movement)
            directions = self.prune_neighbors(current, parent)
            
            for direction in directions:
                neighbor = self.move(current, direction)
                if neighbor and self.is_valid(*neighbor) and neighbor not in closed_set:
                    tentative_g = g + 1  # Cost of 1 for each step
                    
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        g_score[neighbor] = tentative_g
                        f_score = tentative_g + self.cube_distance(neighbor, goal)
                        heapq.heappush(open_set, (f_score, tentative_g, neighbor, current))
                        came_from[neighbor] = current
        
        return []

    def find_path(self, start, goal, algorithm="jps"):
        """Main pathfinding method that delegates to the appropriate algorithm."""
        if algorithm == "bfs":
            return self.find_path_bfs(start, goal)
        elif algorithm == "dijkstra":
            return self.find_path_dijkstra(start, goal)
        elif algorithm == "astar":
            return self.find_path_astar(start, goal)
        elif algorithm == "jps":
            return self.find_path_jps(start, goal)
        else:
            return self.find_path_jps(start, goal)  # Default to JPS

# OLD VISUALIZATION CODE (modified to get rid of optional orientation)

#Initialize Pygame
pygame.init()

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (128, 128, 128)
LIGHT_GRAY = (200, 200, 200)
DARK_GRAY = (64, 64, 64)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
ORANGE = (255, 165, 0)
PURPLE = (128, 0, 128)
CYAN = (0, 255, 255)
PINK = (255, 192, 203)
LIGHT_BLUE = (173, 216, 230)
LIGHT_GREEN = (144, 238, 144)
LIGHT_YELLOW = (255, 255, 224)


class HexGameVisualization:
    def __init__(self, width=1200, height=800):
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("A* Pathfinding Step-by-Step Visualization")
        
        # Grid settings
        self.hex_size = 30
        self.grid_cols = 20
        self.grid_rows = 10

        # Initialize grid with some obstacles
        self.grid = [[0 for _ in range(self.grid_cols)] for _ in range(self.grid_rows)]    
        self.pathfinder = HexGrid(self.grid)
        
        # Add some initial obstacles
        self.add_random_obstacles()
        
        # Game state
        self.start_pos = None
        self.goal_pos = None
        self.path = []
        self.mode = "SET_START"  # SET_START, SET_GOAL, SET_OBSTACLES, PATHFIND
        
        # A* Visualization state
        self.visited_cells = set()
        self.frontier_cells = set()
        self.current_cell = None
        self.current_path = []
        self.step_mode = False
        self.auto_step = False
        self.step_delay = 100  # milliseconds
        self.last_step_time = 0
        
        # Animation
        self.path_animation_progress = 0
        self.animating_path = False
        
        # Algorithm selection
        self.current_algorithm = "astar"  # Focus on A* for visualization
        self.algorithms = ["jps", "bfs", "dijkstra", "astar"]
        self.algorithm_names = {
            "jps": "Jump Point Search",
            "bfs": "Breadth-First Search", 
            "dijkstra": "Dijkstra",
            "astar": "A*"
        }
        
        # Timer
        self.pathfinding_time = None
        self.algorithm_times = {}  # Store times for all algorithms
    
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 18)
        self.clock = pygame.time.Clock()

    def hex_to_pixel(self, row, col):
        """Convert hex grid coordinates to pixel coordinates."""
        size = self.hex_size
        
        x = size * (3/2 * col) + 100
        y = size * (math.sqrt(3) * (row + 0.5 * (col & 1))) + 100
        
        return int(x), int(y)
    
    def pixel_to_hex(self, x, y):
        """Convert pixel coordinates to hex grid coordinates."""
        min_dist = float('inf')
        closest_hex = None
        
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                hx, hy = self.hex_to_pixel(row, col)
                dist = math.sqrt((x - hx) ** 2 + (y - hy) ** 2)
                if dist < min_dist and dist < self.hex_size:
                    min_dist = dist
                    closest_hex = (row, col)
        
        return closest_hex
    
    def draw_hexagon(self, surface, center_x, center_y, size, color, outline_color=BLACK):
        """Draw a single hexagon."""
        points = []
        for i in range(6):
            angle = math.pi / 3 * i
            x = center_x + size * math.cos(angle)
            y = center_y + size * math.sin(angle)
            points.append((x, y))
        
        pygame.draw.polygon(surface, color, points)
        pygame.draw.polygon(surface, outline_color, points, 2)
    
    def draw_grid(self):
        """Draw the hexagonal grid with A* visualization."""
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                x, y = self.hex_to_pixel(row, col)
                
                # Determine hex color based on A* state
                if self.grid[row][col] == 1:  # Obstacle
                    color = DARK_GRAY
                elif (row, col) == self.start_pos:
                    color = GREEN
                elif (row, col) == self.goal_pos:
                    color = RED
                elif (row, col) == self.current_cell and self.step_mode:
                    color = ORANGE  # Currently processing cell
                elif (row, col) in self.visited_cells:
                    color = LIGHT_BLUE  # Visited cells
                elif (row, col) in self.frontier_cells:
                    color = LIGHT_YELLOW  # Frontier cells
                elif (row, col) in self.path:
                    # Final path color
                    if self.animating_path:
                        path_index = self.path.index((row, col))
                        if path_index < self.path_animation_progress:
                            color = YELLOW
                        else:
                            color = BLUE
                    else:
                        color = YELLOW
                else:
                    color = WHITE
                
                self.draw_hexagon(self.screen, x, y, self.hex_size, color)
                
                # Draw current path during step mode
                if self.step_mode and (row, col) in self.current_path:
                    # Draw a smaller hexagon to show current path
                    self.draw_hexagon(self.screen, x, y, self.hex_size * 0.6, PINK, PURPLE)
    
    def draw_ui(self):
        """Draw user interface elements."""
        ui_texts = [
            f"Mode: {self.mode}",
            f"Algorithm: {self.algorithm_names[self.current_algorithm]}",
            "Left Click: Set start/goal/toggle obstacles",
            "Right Click: Clear cell",
            "SPACE: Find path (instant)",
            "V: Visualize A* step-by-step",
            "A: Auto-step through A* (toggle)",
            "N: Next step (manual)",
            "UP/DOWN: Adjust step delay",
            "1-4: Switch algorithm",
            "R: Reset",
            "ESC: Quit"
        ]
        
        for i, text in enumerate(ui_texts):
            color = WHITE if i == 0 or i == 1 else LIGHT_GRAY
            text_surface = self.font.render(text, True, color)
            self.screen.blit(text_surface, (10, 10 + i * 20))
        
        # Show step mode info
        if self.step_mode:
            step_info = [
                f"Step Mode: {'AUTO' if self.auto_step else 'MANUAL'}",
                f"Step Delay: {self.step_delay}ms",
                f"Visited: {len(self.visited_cells)} cells",
                f"Frontier: {len(self.frontier_cells)} cells"
            ]
            
            for i, text in enumerate(step_info):
                text_surface = self.small_font.render(text, True, CYAN)
                self.screen.blit(text_surface, (self.width - 250, 10 + i * 20))
        
        # Show path info
        if self.path:
            path_info = f"Path length: {len(self.path)} steps"
            if self.start_pos and self.goal_pos:
                hex_dist = self.pathfinder.cube_distance(self.start_pos, self.goal_pos)
                path_info += f" | Hex distance: {hex_dist:.1f}"
            
            text_surface = self.font.render(path_info, True, CYAN)
            self.screen.blit(text_surface, (10, self.height - 80))
            
            # Show execution time
            if self.pathfinding_time is not None:
                time_info = f"{self.algorithm_names[self.current_algorithm]} execution time: {self.pathfinding_time:.6f} seconds"
                time_surface = self.font.render(time_info, True, CYAN)
                self.screen.blit(time_surface, (10, self.height - 60))
        
        # Legend
        legend_items = [
            (GREEN, "Start"),
            (RED, "Goal"),
            (LIGHT_BLUE, "Visited"),
            (LIGHT_YELLOW, "Frontier"),
            (ORANGE, "Current"),
            (PINK, "Path"),
            (YELLOW, "Final Path"),
            (DARK_GRAY, "Obstacle")
        ]
        
        legend_x = self.width - 150
        legend_y = self.height - 200
        
        legend_title = self.font.render("Legend:", True, WHITE)
        self.screen.blit(legend_title, (legend_x, legend_y))
        
        for i, (color, label) in enumerate(legend_items):
            y_pos = legend_y + 25 + i * 20
            pygame.draw.circle(self.screen, color, (legend_x + 10, y_pos + 8), 8)
            pygame.draw.circle(self.screen, BLACK, (legend_x + 10, y_pos + 8), 8, 2)
            text_surface = self.small_font.render(label, True, WHITE)
            self.screen.blit(text_surface, (legend_x + 25, y_pos))
    
    def handle_click(self, pos, button):
        """Handle mouse clicks."""
        hex_pos = self.pixel_to_hex(pos[0], pos[1])
        if not hex_pos:
            return
        
        row, col = hex_pos
        
        if button == 1:  # Left click
            if self.mode == "SET_START":
                self.start_pos = (row, col)
                self.grid[row][col] = 0  # Make sure start is walkable
                self.mode = "SET_GOAL"
                self.reset_visualization()
            elif self.mode == "SET_GOAL":
                self.goal_pos = (row, col)
                self.grid[row][col] = 0  # Make sure goal is walkable
                self.mode = "SET_OBSTACLES"
                self.reset_visualization()
            elif self.mode == "SET_OBSTACLES":
                if (row, col) not in [self.start_pos, self.goal_pos]:
                    self.grid[row][col] = 1 - self.grid[row][col]  # Toggle obstacle
                    self.reset_visualization()
        
        elif button == 3:  # Right click
            if (row, col) not in [self.start_pos, self.goal_pos]:
                self.grid[row][col] = 0  # Clear obstacle
                self.reset_visualization()
    
    def reset_visualization(self):
        """Reset A* visualization state."""
        self.visited_cells.clear()
        self.frontier_cells.clear()
        self.current_cell = None
        self.current_path = []
        self.step_mode = False
        self.auto_step = False
        self.path = []
        self.animating_path = False
        self.path_animation_progress = 0
    
    def visualization_callback(self, current, visited, frontier, path):
        """Callback function for step-by-step A* visualization."""
        self.current_cell = current
        self.visited_cells = visited.copy()
        self.frontier_cells = frontier.copy()
        self.current_path = path.copy()
        
        # If in manual mode, wait for user input
        if not self.auto_step:
            return
        
        # If in auto mode, add delay
        current_time = pygame.time.get_ticks()
        if current_time - self.last_step_time < self.step_delay:
            pygame.time.wait(self.step_delay - (current_time - self.last_step_time))
        
        self.last_step_time = pygame.time.get_ticks()
        
        # Draw current state
        self.screen.fill(BLACK)
        self.draw_grid()
        self.draw_ui()
        pygame.display.flip()
    
    def find_path(self):
        """Find path using current algorithm (instant)."""
        if self.start_pos and self.goal_pos:
            self.pathfinder = HexGrid(self.grid)
            
            # Start timer
            start_time = time.time()
            
            self.path = self.pathfinder.find_path(self.start_pos, self.goal_pos, self.current_algorithm)
            
            # End timer and calculate execution time
            end_time = time.time()
            self.pathfinding_time = end_time - start_time
            
            # Print to console
            print(f"{self.algorithm_names[self.current_algorithm]} completed in {self.pathfinding_time:.6f} seconds")
            
            if self.path:
                self.animating_path = True
                self.path_animation_progress = 0
    
    def visualize_astar(self):
        """Start A* step-by-step visualization."""
        if not self.start_pos or not self.goal_pos:
            return
        
        self.reset_visualization()
        self.step_mode = True
        self.pathfinder = HexGrid(self.grid)
        
        # Start timer
        start_time = time.time()
        
        # Run A* with visualization callback
        self.path = self.pathfinder.find_path_astar_stepwise(
            self.start_pos, self.goal_pos, self.visualization_callback
        )
        
        # End timer
        end_time = time.time()
        self.pathfinding_time = end_time - start_time
        
        print(f"A* visualization completed in {self.pathfinding_time:.6f} seconds")
        
        # Clear current cell indicator when done
        self.current_cell = None
        
        if self.path:
            self.animating_path = True
            self.path_animation_progress = 0
    
    def update_animation(self):
        """Update path animation."""
        if self.animating_path:
            self.path_animation_progress += 0.3
            if self.path_animation_progress >= len(self.path):
                self.animating_path = False
    
    def add_random_obstacles(self):
        """Add some random obstacles to the grid."""
        # Add a few random obstacles (about 15% of cells)
        num_obstacles = (self.grid_rows * self.grid_cols) // 7
        for _ in range(num_obstacles):
            row = random.randint(0, self.grid_rows - 1)
            col = random.randint(0, self.grid_cols - 1)
            self.grid[row][col] = 1
    
    def reset(self):
        """Reset the game state."""
        self.start_pos = None
        self.goal_pos = None
        self.path = []
        self.mode = "SET_START"
        self.pathfinding_time = None
        self.algorithm_times = {}
        
        # Reset visualization
        self.reset_visualization()
        
        # Clear grid but keep some obstacles
        self.grid = [[0 for _ in range(self.grid_cols)] for _ in range(self.grid_rows)]
        self.add_random_obstacles()
    
    def run(self):
        """Main game loop."""
        running = True
        
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        self.find_path()
                    elif event.key == pygame.K_v:
                        self.visualize_astar()
                    elif event.key == pygame.K_a:
                        self.auto_step = not self.auto_step
                        print(f"Auto-step: {'ON' if self.auto_step else 'OFF'}")
                    elif event.key == pygame.K_n:
                        if self.step_mode and not self.auto_step:
                            # Manual step - this would need to be implemented
                            # with a generator or state machine for proper stepping
                            pass
                    elif event.key == pygame.K_UP:
                        self.step_delay = max(10, self.step_delay - 10)
                        print(f"Step delay: {self.step_delay}ms")
                    elif event.key == pygame.K_DOWN:
                        self.step_delay = min(1000, self.step_delay + 10)
                        print(f"Step delay: {self.step_delay}ms")
                    elif event.key == pygame.K_r:
                        self.reset()
                    elif event.key == pygame.K_1:
                        self.current_algorithm = "jps"
                        print(f"Switched to {self.algorithm_names[self.current_algorithm]}")
                    elif event.key == pygame.K_2:
                        self.current_algorithm = "bfs"
                        print(f"Switched to {self.algorithm_names[self.current_algorithm]}")
                    elif event.key == pygame.K_3:
                        self.current_algorithm = "dijkstra"
                        print(f"Switched to {self.algorithm_names[self.current_algorithm]}")
                    elif event.key == pygame.K_4:
                        self.current_algorithm = "astar"
                        print(f"Switched to {self.algorithm_names[self.current_algorithm]}")
                
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    self.handle_click(pygame.mouse.get_pos(), event.button)
            
            # Update
            self.update_animation()
            
            # Draw
            self.screen.fill(BLACK)
            self.draw_grid()
            self.draw_ui()
            
            pygame.display.flip()
            self.clock.tick(60)
        
        pygame.quit()
        sys.exit()


if __name__ == "__main__":
    game = HexGameVisualization()
    game.run()