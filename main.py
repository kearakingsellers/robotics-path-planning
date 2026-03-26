import heapq
import matplotlib.pyplot as plt

# =========================
# NODE CLASS
# =========================
class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent

        self.g = 0  # cost from start
        self.h = 0  # heuristic to goal
        self.f = 0  # total cost

    def __lt__(self, other):
        return self.f < other.f


# =========================
# HEURISTIC (MANHATTAN)
# =========================
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# =========================
# GET NEIGHBORS
# =========================
def get_neighbors(node, grid):
    neighbors = []
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    for d in directions:
        row = node.position[0] + d[0]
        col = node.position[1] + d[1]

        if 0 <= row < len(grid) and 0 <= col < len(grid[0]):
            if grid[row][col] == 0:
                neighbors.append((row, col))

    return neighbors


# =========================
# A* ALGORITHM
# =========================
def astar(grid, start, goal):
    open_list = []
    closed_set = set()

    start_node = Node(start)
    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)
        closed_set.add(current_node.position)

        # Goal reached
        if current_node.position == goal:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        for neighbor_pos in get_neighbors(current_node, grid):
            if neighbor_pos in closed_set:
                continue

            neighbor = Node(neighbor_pos, current_node)

            neighbor.g = current_node.g + 1
            neighbor.h = heuristic(neighbor_pos, goal)
            neighbor.f = neighbor.g + neighbor.h

            heapq.heappush(open_list, neighbor)

    return None


# =========================
# VISUALIZATION
# =========================
def visualize(grid, path, start, goal):
    rows = len(grid)
    cols = len(grid[0])

    plt.figure(figsize=(6, 6))

    # Draw grid
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 1:
                plt.scatter(c, rows - r - 1, s=300, marker='s')  # obstacle
            else:
                plt.scatter(c, rows - r - 1, s=100, alpha=0.2)

    # Draw path
    if path:
        x = [p[1] for p in path]
        y = [rows - p[0] - 1 for p in path]
        plt.plot(x, y, linewidth=4)

    # Start and Goal
    plt.scatter(start[1], rows - start[0] - 1, s=400, marker='o')
    plt.scatter(goal[1], rows - goal[0] - 1, s=400, marker='X')

    plt.title("A* Path Planning")
    plt.grid(True)
    plt.savefig("output.png")
    plt.show()
    def visualize(grid, path, start, goal):
    rows = len(grid)
    cols = len(grid[0])

    plt.figure(figsize=(6, 6))

    # Draw grid
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 1:
                plt.scatter(c, rows - r - 1, s=300, marker='s')  # obstacle
            else:
                plt.scatter(c, rows - r - 1, s=100, alpha=0.2)

    # Draw path
    if path:
        x = [p[1] for p in path]
        y = [rows - p[0] - 1 for p in path]
        plt.plot(x, y, linewidth=4)

    # Start and Goal
    plt.scatter(start[1], rows - start[0] - 1, s=400, marker='o')
    plt.scatter(goal[1], rows - goal[0] - 1, s=400, marker='X')

    plt.title("A* Path Planning")
    plt.grid(True)
    plt.savefig("output.png")
    plt.show()
    print("Path length:", len(path))


# =========================
# MAIN TEST
# =========================
if __name__ == "__main__":
    grid = [
        [0, 0, 0, 0, 1],
        [1, 1, 0, 1, 0],
        [0, 0, 0, 1, 0],
        [0, 1, 1, 0, 0],
        [0, 0, 0, 0, 0]
    ]

    start = (0, 0)
    goal = (4, 4)

    path = astar(grid, start, goal)

    print("Path:", path)

    visualize(grid, path, start, goal)