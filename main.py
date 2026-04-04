import math
import heapq
import matplotlib.pyplot as plt
import numpy as np

# =========================
# NODE CLASS
# =========================
class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0

    def __lt__(self, other):
        return self.f < other.f


# =========================
# HEURISTIC
# =========================
def heuristic(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


# =========================
# NEIGHBORS
# =========================
def get_neighbors(node, grid):
    directions = [
        (-1,0),(1,0),(0,-1),(0,1),
        (-1,-1),(-1,1),(1,-1),(1,1)
    ]

    neighbors = []
    for d in directions:
        r = node.position[0] + d[0]
        c = node.position[1] + d[1]

        if 0 <= r < len(grid) and 0 <= c < len(grid[0]):
            if grid[r][c] != 1:
                neighbors.append((r,c))

    return neighbors


# =========================
# COST MAP
# =========================
def create_cost_map(grid):
    rows = len(grid)
    cols = len(grid[0])
    cost_map = [row[:] for row in grid]

    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 1:
                for i in range(-1,2):
                    for j in range(-1,2):
                        nr, nc = r+i, c+j
                        if 0 <= nr < rows and 0 <= nc < cols:
                            if cost_map[nr][nc] == 0:
                                cost_map[nr][nc] = 3

    return cost_map


# =========================
# A*
# =========================
def astar(grid, start, goal):
    open_list = []
    closed = set()

    start_node = Node(start)
    heapq.heappush(open_list, start_node)

    while open_list:
        current = heapq.heappop(open_list)

        if current.position == goal:
            path = []
            while current:
                path.append(current.position)
                current = current.parent
            return path[::-1]

        closed.add(current.position)

        for pos in get_neighbors(current, grid):
            if pos in closed:
                continue

            node = Node(pos, current)

            cell_cost = grid[pos[0]][pos[1]]
            node.g = current.g + 1 + cell_cost * 2
            node.h = heuristic(pos, goal)
            node.f = node.g + node.h

            heapq.heappush(open_list, node)

    return None


# =========================
# SMOOTH PATH
# =========================
def smooth_path(path):
    if not path:
        return path

    smooth = [path[0]]

    for i in range(1, len(path)-1):
        prev = smooth[-1]
        curr = path[i]
        next = path[i+1]

        dir1 = (curr[0]-prev[0], curr[1]-prev[1])
        dir2 = (next[0]-curr[0], next[1]-curr[1])

        if dir1 != dir2:
            smooth.append(curr)

    smooth.append(path[-1])
    return smooth


# =========================
# DYNAMIC OBSTACLE
# =========================
def add_dynamic_obstacle(grid, pos):
    r,c = pos
    if 0 <= r < len(grid) and 0 <= c < len(grid[0]):
        grid[r][c] = 1


# =========================
# VISUALIZATION (HEATMAP)
# =========================
def visualize(grid, path, start, goal, filename):
    grid_array = np.array(grid)

    plt.figure(figsize=(6,6))

    # Heatmap (cost visualization)
    plt.imshow(grid_array, cmap='coolwarm', origin='upper')

    # Obstacles overlay
    for r in range(len(grid)):
        for c in range(len(grid[0])):
            if grid[r][c] == 1:
                plt.gca().add_patch(
                    plt.Rectangle((c-0.5, r-0.5), 1, 1, color='black')
                )

    # Path
    if path:
        x = [p[1] for p in path]
        y = [p[0] for p in path]
        plt.plot(x, y, linewidth=3)

    # Start & Goal
    plt.scatter(start[1], start[0], s=200, label='Start')
    plt.scatter(goal[1], goal[0], s=200, marker='X', label='Goal')

    plt.title("A* Path Planning with Cost Heatmap")
    plt.legend()
    plt.grid(True)

    plt.savefig(filename, bbox_inches='tight')
    plt.show(block=False)
    plt.pause(2)
    plt.close()


# =========================
# MAP
# =========================
def get_map():
    return [
        [0,0,0,0,0,0],
        [0,1,1,0,0,0],
        [0,0,0,0,1,0],
        [0,1,0,0,0,0],
        [0,0,0,1,0,0],
        [0,0,0,0,0,0]
    ]


# =========================
# MAIN
# =========================
if __name__ == "__main__":
    grid = get_map()
    grid = create_cost_map(grid)

    start = (0,0)
    goal = (5,5)

    # First path
    path1 = astar(grid, start, goal)
    path1 = smooth_path(path1)

    visualize(grid, path1, start, goal, "output1.png")

    # Add obstacle
    if path1:
        block = path1[len(path1)//2]
        add_dynamic_obstacle(grid, block)

    # Replan
    path2 = astar(grid, start, goal)
    path2 = smooth_path(path2)

    visualize(grid, path2, start, goal, "output2.png")