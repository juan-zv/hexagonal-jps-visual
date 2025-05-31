# Jump Point Search Visual *Under Development

## Tools used
- [Python](https://www.python.org/)
- [Pygame](https://www.pygame.org/docs/)
- [uv](https://docs.astral.sh/uv/)

## Goal
The goal of this project is to implement Jump Point Search (JPS) for a 2D hexagonal grid using Pygame. JPS is an optimization technique for pathfinding algorithms that reduces the number of nodes that need to be explored.

In a hexagonal grid there are some concerns to address:
- The coordinate system: Hexagonal grids can be represented in different ways, such as axial or offset coordinates. This project uses axial coordinates.
- The movement directions: In a hexagonal grid, there are 6 possible movement directions, which are different from the 8 directions in a square grid.
- The jump points: The algorithm needs to identify jump points based on the movement directions and distances in the hexagonal grid.
- The pruning of branches: The algorithm needs to prune branches that are not necessary to find the shortest path, which is different from the square grid.
- Distances: The distances between nodes in a hexagonal grid are different from those in a square grid, so the algorithm needs to account for this when calculating distances.
- The heuristic: The heuristic used in the A* algorithm needs to be adapted for the hexagonal grid.

## How to run
1. Install the required packages:
   ```python
    pip install uv
   ```
2. Run the virtual environment (this will install required libraries in the virtual environment):
   ```python
    uv run main.py
   ```