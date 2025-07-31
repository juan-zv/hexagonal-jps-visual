import pygame
import heapq
import math
import sys
import random

'''
Note:
    - This code uses cube coordinates (x, y, z) where x + y + z = 0
    - Cube coordinates make hexagonal calculations much simpler and more intuitive
    - The visualization still uses offset coordinates for display purposes
    - 0 is a walkable cell, 1 is an obstacle.
'''

class HexGrid:
    def __init__(self, grid):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if grid else 0
        
        # Cube coordinate directions (6 neighbors)
        self.cube_directions = [
            (1, -1, 0),   # Northeast
            (1, 0, -1),   # East
            (0, 1, -1),   # Southeast
            (-1, 1, 0),   # Southwest
            (-1, 0, 1),   # West
            (0, -1, 1)    # Northwest
        ]
        
    def offset_to_cube(self, row, col):
        """Convert odd-q offset coordinates to cube coordinates."""
        x = col
        z = row - (col - (col & 1)) // 2
        y = -x - z
        return (x, y, z)
    
    def cube_to_offset(self, x, y, z):
        """Convert cube coordinates to odd-q offset coordinates."""
        col = x
        row = z + (x - (x & 1)) // 2
        return (row, col)
    
    def is_valid_cube(self, x, y, z):
        """Check if cube coordinates are valid (within bounds and not an obstacle)."""
        row, col = self.cube_to_offset(x, y, z)
        return (0 <= row < self.rows and
                0 <= col < self.cols and
                self.grid[row][col] != 1)
    
    def is_valid_offset(self, row, col):
        """Check if offset coordinates are valid (within bounds and not an obstacle)."""
        return (0 <= row < self.rows and
                0 <= col < self.cols and
                self.grid[row][col] != 1)
    
    def get_cube_neighbors(self, x, y, z):
        """Get all valid neighbors of a cube coordinate."""
        neighbors = []
        for dx, dy, dz in self.cube_directions:
            nx, ny, nz = x + dx, y + dy, z + dz
            if self.is_valid_cube(nx, ny, nz):
                neighbors.append((nx, ny, nz))
        return neighbors
    
    def cube_distance(self, cube1, cube2):
        """Calculate distance between two cube coordinates."""
        x1, y1, z1 = cube1
        x2, y2, z2 = cube2
        return (abs(x1 - x2) + abs(y1 - y2) + abs(z1 - z2)) // 2
    
    def get_cube_direction(self, from_cube, to_cube):
        """Get the direction index from one cube coordinate to another."""
        fx, fy, fz = from_cube
        tx, ty, tz = to_cube
        direction = (tx - fx, ty - fy, tz - fz)
        
        try:
            return self.cube_directions.index(direction)
        except ValueError:
            return None
    
    def has_forced_neighbor(self, cube_pos):
        """Check if a position has forced neighbors (for jump point search)."""
        x, y, z = cube_pos
        neighbors = self.get_cube_neighbors(x, y, z)
        accessible_neighbors = len(neighbors)
        total_possible = 6  # Hexagons always have 6 potential neighbors
        
        return accessible_neighbors < total_possible and accessible_neighbors >= 3
    
    def jump_in_direction(self, start_cube, direction_idx, goal_cube, max_distance=1):
        """Jump in a specific direction looking for jump points."""
        current = start_cube
        distance = 0
        
        while distance < max_distance:
            # Move in the specified direction
            dx, dy, dz = self.cube_directions[direction_idx]
            next_cube = (current[0] + dx, current[1] + dy, current[2] + dz)
            
            if not self.is_valid_cube(*next_cube):
                return None
            
            current = next_cube
            distance += 1
            
            if current == goal_cube:
                return current
            
            if self.has_forced_neighbor(current):
                return current
        
        return None
    
    def find_path(self, start_offset, goal_offset):
        """Find path using A* with cube coordinates."""
        # Convert to cube coordinates
        start_cube = self.offset_to_cube(*start_offset)
        goal_cube = self.offset_to_cube(*goal_offset)
        
        if not self.is_valid_offset(*start_offset) or not self.is_valid_offset(*goal_offset):
            return []
        
        if start_cube == goal_cube:
            return [start_offset]
        
        open_set = [(0, 0, start_cube)]
        heapq.heapify(open_set)
        
        open_set_tracker = {start_cube}
        closed_set = set()
        came_from = {}
        g_score = {start_cube: 0}
        f_score = {start_cube: self.cube_distance(start_cube, goal_cube)}
        
        while open_set:
            current_entry = heapq.heappop(open_set)
            current_cube = current_entry[2]
            
            if current_cube in closed_set:
                continue
            
            closed_set.add(current_cube)
            
            if current_cube == goal_cube:
                # Reconstruct path in offset coordinates
                path = []
                current = current_cube
                while current in came_from:
                    path.append(self.cube_to_offset(*current))
                    current = came_from[current]
                path.append(start_offset)
                return path[::-1]
            
            neighbors = self.get_cube_neighbors(*current_cube)
            
            for neighbor_cube in neighbors:
                if neighbor_cube in closed_set:
                    continue
                
                # Optional: Jump point search enhancement
                direction_idx = self.get_cube_direction(current_cube, neighbor_cube)
                if direction_idx is not None:
                    jump_point = self.jump_in_direction(current_cube, direction_idx, goal_cube)
                    if jump_point:
                        neighbor_cube = jump_point
                
                if neighbor_cube in closed_set:
                    continue
                
                tentative_g_score = g_score[current_cube] + self.cube_distance(current_cube, neighbor_cube)
                
                if neighbor_cube not in g_score or tentative_g_score < g_score[neighbor_cube]:
                    came_from[neighbor_cube] = current_cube
                    g_score[neighbor_cube] = tentative_g_score
                    f_score[neighbor_cube] = tentative_g_score + self.cube_distance(neighbor_cube, goal_cube)
                    
                    if neighbor_cube not in open_set_tracker:
                        heapq.heappush(open_set, (f_score[neighbor_cube], g_score[neighbor_cube], neighbor_cube))
                        open_set_tracker.add(neighbor_cube)
        
        return []

# VISUALIZATION CODE (mostly unchanged, still uses offset coordinates for display)

# Initialize Pygame
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


class HexGameVisualization:
    def __init__(self, width=1000, height=600):
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Hexagonal Grid Pathfinding with Cube Coordinates")
        
        # Grid settings
        self.hex_size = 30
        self.grid_cols = 7
        self.grid_rows = 7

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
        
        # Animation
        self.path_animation_progress = 0
        self.animating_path = False
    
        self.font = pygame.font.Font(None, 24)
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
        """Draw the hexagonal grid."""
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                x, y = self.hex_to_pixel(row, col)
                
                # Determine hex color
                if self.grid[row][col] == 1:  # Obstacle
                    color = DARK_GRAY
                elif (row, col) == self.start_pos:
                    color = GREEN
                elif (row, col) == self.goal_pos:
                    color = RED
                elif (row, col) in self.path:
                    # Animated path color
                    path_index = self.path.index((row, col))
                    if self.animating_path and path_index < self.path_animation_progress:
                        color = YELLOW
                    elif not self.animating_path and (row, col) in self.path:
                        color = YELLOW
                    else:
                        color = BLUE
                else:
                    color = WHITE
                
                self.draw_hexagon(self.screen, x, y, self.hex_size, color)
                
                # Optional: Draw cube coordinates as text (for debugging)
                if hasattr(self, 'show_coordinates') and self.show_coordinates:
                    cube_coords = self.pathfinder.offset_to_cube(row, col)
                    coord_text = f"{cube_coords[0]},{cube_coords[1]},{cube_coords[2]}"
                    text_surface = pygame.font.Font(None, 16).render(coord_text, True, BLACK)
                    text_rect = text_surface.get_rect(center=(x, y))
                    self.screen.blit(text_surface, text_rect)
    
    def draw_ui(self):
        """Draw user interface elements."""
        # ui_texts = [
        #     f"Mode: {self.mode}",
        #     "Left Click: Set start/goal/toggle obstacles",
        #     "Right Click: Clear cell",
        #     "SPACE: Find path",
        #     "R: Reset",
        #     "C: Toggle coordinate display",
        #     "ESC: Quit"
        # ]
        
        # for i, text in enumerate(ui_texts):
        #     color = WHITE if i == 0 else LIGHT_GRAY
        #     text_surface = self.font.render(text, True, color)
        #     self.screen.blit(text_surface, (10, 10 + i * 25))
        
        # Show path info
        # if self.path:
        #     path_info = f"Path length: {len(self.path)} steps"
        #     if self.start_pos and self.goal_pos:
        #         start_cube = self.pathfinder.offset_to_cube(*self.start_pos)
        #         goal_cube = self.pathfinder.offset_to_cube(*self.goal_pos)
        #         cube_dist = self.pathfinder.cube_distance(start_cube, goal_cube)
        #         path_info += f" | Cube distance: {cube_dist}"
            
        #     text_surface = self.font.render(path_info, True, CYAN)
        #     self.screen.blit(text_surface, (10, self.height - 30))
    
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
            elif self.mode == "SET_GOAL":
                self.goal_pos = (row, col)
                self.grid[row][col] = 0  # Make sure goal is walkable
                self.mode = "SET_OBSTACLES"
            elif self.mode == "SET_OBSTACLES":
                if (row, col) not in [self.start_pos, self.goal_pos]:
                    self.grid[row][col] = 1 - self.grid[row][col]  # Toggle obstacle
        
        elif button == 3:  # Right click
            if (row, col) not in [self.start_pos, self.goal_pos]:
                self.grid[row][col] = 0  # Clear obstacle
    
    def find_path(self):
        """Find and animate path."""
        if self.start_pos and self.goal_pos:
            self.pathfinder = HexGrid(self.grid)
            self.path = self.pathfinder.find_path(self.start_pos, self.goal_pos)
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
        self.animating_path = False
        self.path_animation_progress = 0
        
        # Clear grid but keep some obstacles
        # self.grid = [[0 for _ in range(self.grid_cols)] for _ in range(self.grid_rows)]
        # self.add_random_obstacles()
    
    def run(self):
        """Main game loop."""
        running = True
        self.show_coordinates = False
        
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        self.find_path()
                    elif event.key == pygame.K_r:
                        self.reset()
                    elif event.key == pygame.K_c:
                        self.show_coordinates = not self.show_coordinates
                
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