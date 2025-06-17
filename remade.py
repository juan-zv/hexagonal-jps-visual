import pygame
import heapq
import math
import sys

#Initialize Pygame
pygame.init()

#OddQ vertical layout shoves odd columns down

class HexGrid:
    def __init__(self, grid):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if grid else 0
        self.directions = {
                0: (0, -1),  # North
                1: (+1,-1),  # Northeast
                2: (+1, 0),   # Southeast
                3: (0, +1),   # South
                4: (-1, 0),  # Southwest
                5: (-1, -1)  # Northwest
                }
        
    def is_valid(self, row, col):
        return (0 <= row < self.rows and
                0 <= col < self.cols and
                self.grid[row][col] != 1)

    #TO DO: review this function   
    def get_neighbors(self, row, col):
        neighbors = []

        for direction in self.directions.values():
            if col % 2 != 0:
                if direction == (-1, -1):
                    direction = (-1, 0)                
                elif direction == (-1, 0):
                    direction = (-1, +1)
                elif direction == (+1,-1):
                    direction = (+1, 0)
                elif direction == (+1, 0):
                    direction = (+1, +1)
            else:
                continue

            new_row = row + direction[0]
            new_col = col + direction[1]

            if self.is_valid(new_row, new_col):
                neighbors.append((new_row, new_col))
        
        return neighbors
    
    def odd_q_to_cube(self, a):
        row, col = a
        x = col
        z = row - (col - (col % 1)) // 2 #try operator bitwise AND '&'
        y = -x - z

        return (x, y, z)
    
    def cube_distance(self, a, b):
        a_cube = self.odd_q_to_cube(a)
        b_cube = self.odd_q_to_cube(b)

        return (abs(a_cube[0] - b_cube[0]) +
                abs(a_cube[1] - b_cube[1]) +
                abs(a_cube[2] - b_cube[2])) // 2
    
    #TO DO: double check this function
    def get_directions(self, a, b):
        a_row, a_col = a
        b_row, b_col = b

        dir_row = b_row - a_row
        dir_col = b_col - a_col

        for direction, (expected_dir_row, expected_dir_col) in self.directions.items():
            if dir_row == expected_dir_row and dir_col == expected_dir_col:
                return direction
        return None
    
    #TO DO: Review rules for forced neighbors and DIRECTION
    def has_forced_neighbor(self, pos):
        row, col = pos
        neighbors = self.get_neighbors(row, col)
        all_neighbors = self.directions

        accessible_neighbors = len(neighbors)
        total_possible =  len(all_neighbors)

        return accessible_neighbors < total_possible and accessible_neighbors >= 3
    
    def jump_in_direction(self, start, direction, goal, max_distance=3):
        current = start
        distance = 0

        while distance < max_distance:
            current_row, current_col = current
            neighbors = self.get_neighbors(current_row, current_col)
            next_pos = None

            for neighbor in neighbors:
                if self.get_directions(current, neighbor) == direction:
                    next_pos = neighbor
                    break

            if next_pos is None or not self.is_valid(next_pos[0], next_pos[1]):
                return None
            
            current = next_pos
            distance += 1

            if current == goal:
                return current
            
            if self.has_forced_neighbor(current):
                return current
    
    # FINISH this
    def find_path(self, start, goal):
        start_row, start_col = start
        goal_row, goal_col = goal

        if not self.is_valid(start_row, start_col) or not self.is_valid(goal_col, goal_row):
            return []
        
        if start == goal:
            return [start]
        
        open_set = [(0, 0, start)]
        heapq.heapify(open_set)

        closed_set = set()
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.cube_distance(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)

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

                direction = self.get_directions(current, neighbor)
                if direction is not None:
                    jump_point = self.jump_in_direction(current, direction, goal)
                    if jump_point:
                        neighbor = jump_point

                if neighbor in closed_set:
                    continue

                tentative_g_score = g_score[current] + self.cube_distance(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.cube_distance(neighbor, goal)

                    #added this
                    if neighbor not in [i[2] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], g_score[neighbor], neighbor))

        return []
    
# OLD VISUALIZATION CODE (modified to get rid of optional orientation)

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
    def __init__(self, width=1000, height=700):
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Hexagonal Grid Pathfinding")
        
        # Grid settings
        self.hex_size = 30
        self.grid_cols = 8
        self.grid_rows = 8
        
        # Initialize grid with some obstacles
        self.grid = [[0 for _ in range(self.grid_cols)] for _ in range(self.grid_rows)]    
        self.pathfinder = HexGrid(self.grid)
        
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
                hex_dist = self.pathfinder.cube_distance(self.start_pos, self.goal_pos)
                path_info += f" | Hex distance: {hex_dist:.1f}"
            
            text_surface = self.font.render(path_info, True, CYAN)
            self.screen.blit(text_surface, (10, self.height - 30))
    
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

    