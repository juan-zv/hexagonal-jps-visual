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

In depth explanation of the algorithm can be found in the [Jump Point Search docs/analysis](https://www.notion.so/Jump-Point-Search-for-Hexagonal-Grids-21e45e3d56b28019a2b6e6b966f48161?source=copy_link).

## How to run
1. Install the required packages (uv creates a virtual environment):
   ```python
    pip install uv
   ```
2. (a) Run the virtual environment (this will install required libraries and dependencies in the virtual environment):
   ```python
    uv run main.py
   ```
Note: main.py is the full implementation of the different layouts and approaches to the JPS algorithm in a hexagonal grid. Right now is under development, so it is not fully functional yet. Use the remade.py file to run a minimal implementation of the JPS algorithm in a hexagonal grid. Details in the [Jump Point Search docs/analysis](https://www.notion.so/Jump-Point-Search-for-Hexagonal-Grids-21e45e3d56b28019a2b6e6b966f48161?source=copy_link).

2. (b) Run the minimal implementation:
   ```python
    uv run remade.py
   ```