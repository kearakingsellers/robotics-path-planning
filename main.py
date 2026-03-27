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
# HEURISTIC
# =========================
def heuristic(a, b):
    return ((a[0] - b[0])**2 + (a[1] - b[1])**2) ** 0.5


# =========================
# GET NEIGHBORS (8 directions)
# =========================
def get_neighbors(node, grid):
    directions = [
        (-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1)
    ]

    neighbors = []

    for d in directions:
        r = node.position[0] + d[0]
        c = node.position[1] + d[1]

        if 0 <= r < len(grid) and 0 <= c < len(grid[0]):
            if grid[r][c] == 0:
                neighbors.append((r, c))

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

        if current_node.position == goal:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        for pos in get_neighbors(current_node, grid):
            if pos in closed_set:
                continue

            neighbor = Node(pos, current_node)

            # diagonal vs straight cost
            if pos[0] != current_node.position[0] and pos[1] != current_node.position[1]:
                step_cost = 1.4
            else:
                step_cost = 1

            neighbor.g = current_node.g + step_cost
            neighbor.h = heuristic(pos, goal)
            neighbor.f = neighbor.g + neighbor.h

            heapq.heappush(open_list, neighbor)

    return None


# =========================
# VISUALIZATION
# =========================
def visualize(grid, path, start, goal):
    rows = len(grid)

    plt.figure(figsize=(6, 6))

    for r in range(rows):
        for c in range(len(grid[0])):
            if grid[r][c] == 1:
                plt.scatter(c, rows - r - 1, s=300, marker='s')
            else:
                plt.scatter(c, rows - r - 1, s=100, alpha=0.2)

    if path:
        x = [p[1] for p in path]
        y = [rows - p[0] - 1 for p in path]
        plt.plot(x, y, linewidth=4)

    plt.scatter(start[1], rows - start[0] - 1, s=400, marker='o')
    plt.scatter(goal[1], rows - goal[0] - 1, s=400, marker='X')

    plt.title("A* Path Planning")
    plt.grid(True)
    plt.savefig("output.png", bbox_inches='tight')
    plt.show()


# =========================
# MULTIPLE MAPS
# =========================
def get_map(choice):
    maps = {
        1: [
            [0,0,0,0,1],
            [1,1,0,1,0],
            [0,0,0,1,0],
            [0,1,1,0,0],
            [0,0,0,0,0]
        ],
        2: [
            [0,0,1,0,0],
            [0,1,1,0,1],
            [0,0,0,0,0],
            [1,0,1,1,0],
            [0,0,0,0,0]
        ]
    }
    return maps.get(choice, maps[1])


# =========================
# MAIN SYSTEM
# =========================
if __name__ == "__main__":

    print("Select Map: 1 or 2")
    map_choice = int(input("Enter map number: "))

    grid = get_map(map_choice)

    print("Enter Start Position (row col): ")
    start = tuple(map(int, input().split()))

    print("Enter Goal Position (row col): ")
    goal = tuple(map(int, input().split()))

    path = astar(grid, start, goal)

    if path:
        print("Path:", path)
        print("Path length:", len(path))
        visualize(grid, path, start, goal)
    else:
        print("No path found!")