import math
import heapq
import matplotlib.pyplot as plt

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
# HEURISTIC (EUCLIDEAN)
# =========================
def heuristic(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


# =========================
# GET NEIGHBORS (8-DIRECTION)
# =========================
def get_neighbors(node, grid):
    directions = [
        (-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1)
    ]

    neighbors = []
    for d in directions:
        new_row = node.position[0] + d[0]
        new_col = node.position[1] + d[1]

        if 0 <= new_row < len(grid) and 0 <= new_col < len(grid[0]):
            if grid[new_row][new_col] != 1:
                neighbors.append((new_row, new_col))

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
                for i in range(-1, 2):
                    for j in range(-1, 2):
                        nr, nc = r + i, c + j
                        if 0 <= nr < rows and 0 <= nc < cols:
                            if cost_map[nr][nc] == 0:
                                cost_map[nr][nc] = 3  # risky

    return cost_map


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

        if current_node.position == goal:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        for neighbor_pos in get_neighbors(current_node, grid):
            if neighbor_pos in closed_set:
                continue

            neighbor_node = Node(neighbor_pos, current_node)

            cell_cost = grid[neighbor_pos[0]][neighbor_pos[1]]
            tentative_g = current_node.g + 1 + cell_cost * 2

            neighbor_node.g = tentative_g
            neighbor_node.h = heuristic(neighbor_pos, goal)
            neighbor_node.f = neighbor_node.g + neighbor_node.h

            heapq.heappush(open_list, neighbor_node)

    return None


# =========================
# PATH SMOOTHING
# =========================
def smooth_path(path):
    if not path:
        return path

    smooth = [path[0]]

    for i in range(1, len(path) - 1):
        prev = smooth[-1]
        curr = path[i]
        next = path[i + 1]

        dir1 = (curr[0] - prev[0], curr[1] - prev[1])
        dir2 = (next[0] - curr[0], next[1] - curr[1])

        if dir1 != dir2:
            smooth.append(curr)

    smooth.append(path[-1])
    return smooth


# =========================
# DYNAMIC OBSTACLE
# =========================
def add_dynamic_obstacle(grid, position):
    r, c = position
    if 0 <= r < len(grid) and 0 <= c < len(grid[0]):
        grid[r][c] = 1


# =========================
# VISUALIZATION
# =========================
def visualize(grid, path, start, goal, filename):
    plt.figure(figsize=(6, 6))

    for r in range(len(grid)):
        for c in range(len(grid[0])):
            if grid[r][c] == 1:
                plt.scatter(c, r, color='black', s=200)
            elif grid[r][c] > 1:
                plt.scatter(c, r, color='gray', s=100)

    if path:
        x = [p[1] for p in path]
        y = [p[0] for p in path]
        plt.plot(x, y, marker='o')

    plt.scatter(start[1], start[0], color='green', label='Start', s=100)
    plt.scatter(goal[1], goal[0], color='red', label='Goal', s=100)

    plt.gca().invert_yaxis()
    plt.legend()
    plt.title("A* Path Planning (Smoothed + Cost Map)")

    plt.savefig(filename, bbox_inches='tight')
    plt.show(block=False)
    plt.pause(2)
    plt.close()


# =========================
# MAPS
# =========================
def get_map(choice):
    maps = {
        1: [
            [0,0,0,0,0],
            [0,1,1,0,0],
            [0,0,0,0,0],
            [0,1,0,1,0],
            [0,0,0,0,0]
        ]
    }
    return maps.get(choice, maps[1])


# =========================
# MAIN
# =========================
if __name__ == "__main__":
    print("Select Map: 1")
    try:
        map_choice = int(input("Enter map number: "))
    except:
        map_choice = 1

    base_grid = get_map(map_choice)
    grid = create_cost_map(base_grid)

    start = (0, 1)
    goal = (len(grid)-1, len(grid[0])-2)

    # FIRST PATH
    path1 = astar(grid, start, goal)
    path1 = smooth_path(path1)

    print("Initial Path:", path1)
    visualize(grid, path1, start, goal, "output1.png")

    # ADD DYNAMIC OBSTACLE
    if path1 and len(path1) > 3:
        block = path1[len(path1)//2]
        print("Adding obstacle at:", block)
        add_dynamic_obstacle(grid, block)

    # REPLAN
    path2 = astar(grid, start, goal)
    path2 = smooth_path(path2)

    print("New Path:", path2)
    visualize(grid, path2, start, goal, "output2.png")