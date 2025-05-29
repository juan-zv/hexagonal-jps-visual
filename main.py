import pygame
import heapq
import math
from typing import List, Tuple, Optional, Set, Dict
from enum import Enum
import sys

# Initialize Pygame
pygame.init()

# Colors
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

class HexOrientation(Enum):
    POINTY_TOP = "pointy"
    FLAT_TOP = "flat"

class HexGrid:
    def __init__(self, grid: List[List[int]], orientation: HexOrientation = HexOrientation.POINTY_TOP):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if grid else 0
        self.orientation = orientation
        
        # Hexagonal directions (6 neighbors)
        if orientation == HexOrientation.POINTY_TOP:
            self.directions = {
                0: (0, 1),   # East
                1: (-1, 1),  # Northeast (odd rows) / Southeast (even rows)
                2: (-1, 0),  # Northwest (odd rows) / Southwest (even rows)  
                3: (0, -1),  # West
                4: (1, 0),   # Southwest (odd rows) / Northwest (even rows)
                5: (1, 1)    # Southeast (odd rows) / Northeast (even rows)
            }
        else:
            self.directions = {
                0: (-1, 0),  # North
                1: (-1, 1),  # Northeast
                2: (1, 1),   # Southeast
                3: (1, 0),   # South
                4: (1, -1),  # Southwest
                5: (-1, -1)  # Northwest
            }
    
    def get_neighbors(self, row: int, col: int) -> List[Tuple[int, int]]:
        neighbors = []
        
        for direction, (dr, dc) in self.directions.items():
            if self.orientation == HexOrientation.POINTY_TOP:
                if row % 2 == 1:  # Odd row
                    new_row = row + dr
                    new_col = col + dc
                else:  # Even row
                    new_row = row + dr
                    new_col = col + dc
                    if dr != 0:
                        new_col -= 1
            else:
                new_row = row + dr
                new_col = col + dc
                if col % 2 == 1 and dr != 0:
                    new_row += 1 if dr > 0 else -1
            
            if self.is_valid(new_row, new_col):
                neighbors.append((new_row, new_col))
        
        return neighbors
    
    def is_valid(self, row: int, col: int) -> bool:
        return (0 <= row < self.rows and 
                0 <= col < self.cols and 
                self.grid[row][col] == 0)
    
    def hex_distance(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        r1, c1 = a
        r2, c2 = b
        
        if self.orientation == HexOrientation.POINTY_TOP:
            def oddq_to_cube(row, col):
                x = col
                z = row - (col - (col & 1)) // 2
                y = -x - z
                return x, y, z
            
            x1, y1, z1 = oddq_to_cube(r1, c1)
            x2, y2, z2 = oddq_to_cube(r2, c2)
        else:
            def evenq_to_cube(row, col):
                x = col - (row - (row & 1)) // 2
                z = row
                y = -x - z
                return x, y, z
            
            x1, y1, z1 = evenq_to_cube(r1, c1)
            x2, y2, z2 = evenq_to_cube(r2, c2)
        
        return (abs(x1 - x2) + abs(y1 - y2) + abs(z1 - z2)) / 2
    
    def get_direction(self, from_pos: Tuple[int, int], to_pos: Tuple[int, int]) -> Optional[int]:
        dr = to_pos[0] - from_pos[0]
        dc = to_pos[1] - from_pos[1]
        
        for direction, (expected_dr, expected_dc) in self.directions.items():
            if self.orientation == HexOrientation.POINTY_TOP:
                if from_pos[0] % 2 == 0 and expected_dr != 0:
                    expected_dc -= 1
                
                if dr == expected_dr and dc == expected_dc:
                    return direction
            else:
                if from_pos[1] % 2 == 1 and expected_dr != 0:
                    expected_dr += 1 if expected_dr > 0 else -1
                
                if dr == expected_dr and dc == expected_dc:
                    return direction
        
        return None
    
    def jump_in_direction(self, start: Tuple[int, int], direction: int, 
                         goal: Tuple[int, int], max_distance: int = 3) -> Optional[Tuple[int, int]]:
        current = start
        distance = 0
        
        while distance < max_distance:
            neighbors = self.get_neighbors(current[0], current[1])
            next_pos = None
            
            for neighbor in neighbors:
                if self.get_direction(current, neighbor) == direction:
                    next_pos = neighbor
                    break
            
            if next_pos is None or not self.is_valid(next_pos[0], next_pos[1]):
                return None
            
            current = next_pos
            distance += 1
            
            if current == goal:
                return current
            
            if self.has_forced_neighbors(current, direction):
                return current
        
        return current
    
    def has_forced_neighbors(self, pos: Tuple[int, int], came_from_direction: int) -> bool:
        neighbors = self.get_neighbors(pos[0], pos[1])
        all_neighbors = self.get_all_hex_neighbors(pos[0], pos[1])
        
        accessible_count = len(neighbors)
        total_possible = len([n for n in all_neighbors if 0 <= n[0] < self.rows and 0 <= n[1] < self.cols])
        
        return accessible_count < total_possible and accessible_count >= 3
    
    def get_all_hex_neighbors(self, row: int, col: int) -> List[Tuple[int, int]]:
        neighbors = []
        
        for direction, (dr, dc) in self.directions.items():
            if self.orientation == HexOrientation.POINTY_TOP:
                if row % 2 == 1:
                    new_row = row + dr
                    new_col = col + dc
                else:
                    new_row = row + dr
                    new_col = col + dc
                    if dr != 0:
                        new_col -= 1
            else:
                new_row = row + dr
                new_col = col + dc
                if col % 2 == 1 and dr != 0:
                    new_row += 1 if dr > 0 else -1
            
            neighbors.append((new_row, new_col))
        
        return neighbors
    
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        if not self.is_valid(start[0], start[1]) or not self.is_valid(goal[0], goal[1]):
            return []
        
        if start == goal:
            return [start]
        
        open_set = [(0, 0, start)]
        heapq.heapify(open_set)
        
        closed_set: Set[Tuple[int, int]] = set()
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_score = {start: 0}
        f_score = {start: self.hex_distance(start, goal)}
        
        while open_set:
            current_f, current_g, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]
            
            neighbors = self.get_neighbors(current[0], current[1])
            
            for neighbor in neighbors:
                if neighbor in closed_set:
                    continue
                
                direction = self.get_direction(current, neighbor)
                if direction is not None:
                    jump_point = self.jump_in_direction(current, direction, goal)
                    if jump_point:
                        neighbor = jump_point
                
                if neighbor in closed_set:
                    continue
                
                tentative_g = g_score[current] + self.hex_distance(current, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.hex_distance(neighbor, goal)
                    
                    heapq.heappush(open_set, (f_score[neighbor], tentative_g, neighbor))
        
        return []


class HexGameVisualization:
    def __init__(self, width=1000, height=700):
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Hexagonal Grid Pathfinding")
        
        # Grid settings
        self.hex_size = 25
        self.grid_cols = 16
        self.grid_rows = 12
        
        # Initialize grid with some obstacles
        self.grid = [[0 for _ in range(self.grid_cols)] for _ in range(self.grid_rows)]
        # self.add_random_obstacles()
        
        self.pathfinder = HexGrid(self.grid, HexOrientation.POINTY_TOP)
        
        # Game state
        self.start_pos = None
        self.goal_pos = None
        self.path = []
        self.mode = "SET_START"  # SET_START, SET_GOAL, SET_OBSTACLES, PATHFIND
        
        # Animation
        self.path_animation_progress = 0
        self.animating_path = False
        
        # Font for UI
        self.font = pygame.font.Font(None, 24)
        
        self.clock = pygame.time.Clock()
    
    def add_random_obstacles(self):
        """Add some predefined obstacles for demonstration."""
        obstacles = [
            (2, 3), (2, 4), (3, 4), (4, 4), (5, 4),
            (7, 8), (7, 9), (8, 8), (8, 9),
            (1, 10), (2, 10), (3, 11), (4, 11),
            (9, 2), (9, 3), (10, 2), (10, 3)
        ]
        
        for row, col in obstacles:
            if 0 <= row < self.grid_rows and 0 <= col < self.grid_cols:
                self.grid[row][col] = 1
    
    def hex_to_pixel(self, row: int, col: int) -> Tuple[int, int]:
        """Convert hex grid coordinates to pixel coordinates."""
        size = self.hex_size
        
        # Pointy-top hexagon pixel coordinates
        x = size * (3/2 * col) + 100
        y = size * (math.sqrt(3) * (row + 0.5 * (col & 1))) + 100
        
        return int(x), int(y)
    
    def pixel_to_hex(self, x: int, y: int) -> Optional[Tuple[int, int]]:
        """Convert pixel coordinates to hex grid coordinates."""
        # Approximate conversion - find closest hex center
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
    
    def draw_hexagon(self, surface, center_x: int, center_y: int, size: int, color, outline_color=BLACK):
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
    
    def draw_ui(self):
        """Draw user interface elements."""
        ui_texts = [
            f"Mode: {self.mode}",
            "Left Click: Set start/goal/toggle obstacles",
            "Right Click: Clear cell",
            "SPACE: Find path",
            "R: Reset",
            "ESC: Quit"
        ]
        
        for i, text in enumerate(ui_texts):
            color = WHITE if i == 0 else LIGHT_GRAY
            text_surface = self.font.render(text, True, color)
            self.screen.blit(text_surface, (10, 10 + i * 25))
        
        # Show path info
        if self.path:
            path_info = f"Path length: {len(self.path)} steps"
            if self.start_pos and self.goal_pos:
                hex_dist = self.pathfinder.hex_distance(self.start_pos, self.goal_pos)
                path_info += f" | Hex distance: {hex_dist:.1f}"
            
            text_surface = self.font.render(path_info, True, CYAN)
            self.screen.blit(text_surface, (10, self.height - 30))
    
    def handle_click(self, pos: Tuple[int, int], button: int):
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
            self.pathfinder = HexGrid(self.grid, HexOrientation.POINTY_TOP)
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
    
    def reset(self):
        """Reset the game state."""
        self.start_pos = None
        self.goal_pos = None
        self.path = []
        self.mode = "SET_START"
        self.animating_path = False
        self.path_animation_progress = 0
        
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
                    elif event.key == pygame.K_r:
                        self.reset()
                
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