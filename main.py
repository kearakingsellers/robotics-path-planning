import heapq
import math
import matplotlib.pyplot as plt

# ==============================
# NODE CLASS
# ==============================

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent

        self.g = 0  # cost from start
        self.h = 0  # heuristic to goal
        self.f = 0  # total cost

    def __lt__(self, other):
        return self.f < other.f


# ==============================
# HEURISTIC (EUCLIDEAN)
# ==============================

def heuristic(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


# ==============================
# MOVEMENT COST
# ==============================

def movement_cost(current, neighbor):
    # Diagonal movement
    if current[0] != neighbor[0] and current[1] != neighbor[1]:
        return math.sqrt(2)
    return 1


# ==============================
# GET NEIGHBORS (8 DIRECTIONS)
# ==============================

def get_neighbors(node, grid):
    directions = [
        (-1, 0), (1, 0), (0, -1), (0, 1),   # straight
        (-1, -1), (-1, 1), (1, -1), (1, 1)  # diagonal
    ]

    neighbors = []

    for d in directions:
        new_row = node.position[0] + d[0]
        new_col = node.position[1] + d[1]

        if 0 <= new_row < len(grid) and 0 <= new_col < len(grid[0]):
            if grid[new_row][new_col] == 0:
                neighbors.append((new_row, new_col))

    return neighbors


# ==============================
# RECONSTRUCT PATH
# ==============================

def reconstruct_path(current_node):
    path = []
    while current_node:
        path.append(current_node.position)
        current_node = current_node.parent
    return path[::-1]


# ==============================
# A* ALGORITHM
# ==============================

def astar(grid, start, goal):
    open_list = []
    closed_set = set()

    start_node = Node(start)
    goal_node = Node(goal)

    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.position == goal_node.position:
            return reconstruct_path(current_node)

        closed_set.add(current_node.position)

        for neighbor_pos in get_neighbors(current_node, grid):

            if neighbor_pos in closed_set:
                continue

            neighbor_node = Node(neighbor_pos, current_node)

            # Cost calculations
            neighbor_node.g = current_node.g + movement_cost(current_node.position, neighbor_pos)
            neighbor_node.h = heuristic(neighbor_pos, goal)
            neighbor_node.f = neighbor_node.g + neighbor_node.h

            # Avoid duplicates in open list
            if any(n.position == neighbor_node.position and n.f <= neighbor_node.f for n in open_list):
                continue

            heapq.heappush(open_list, neighbor_node)

    return None


# ==============================
# VISUALIZATION
# ==============================

def visualize(grid, path, start, goal):
    rows = len(grid)
    cols = len(grid[0])

    fig, ax = plt.subplots()

    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 1:
                ax.add_patch(plt.Rectangle((j, i), 1, 1, color='black'))

    # Draw path
    if path:
        x = [p[1] + 0.5 for p in path]
        y = [p[0] + 0.5 for p in path]
        ax.plot(x, y, marker='o')

    # Start & Goal
    ax.plot(start[1] + 0.5, start[0] + 0.5, "go", label="Start")
    ax.plot(goal[1] + 0.5, goal[0] + 0.5, "ro", label="Goal")

    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)
    ax.set_aspect('equal')
    ax.invert_yaxis()
    plt.legend()
    plt.title("A* Path Planning (Diagonal + Euclidean)")
    plt.show()


# ==============================
# MAP SELECTION
# ==============================

def get_map(choice):
    maps = {
        1: [
            [0,0,0,0,0],
            [1,1,0,1,0],
            [0,0,0,1,0],
            [0,1,0,0,0],
            [0,0,0,1,0]
        ],
        2: [
            [0,0,0,0,0,0],
            [0,1,1,1,1,0],
            [0,0,0,0,1,0],
            [1,1,1,0,1,0],
            [0,0,0,0,0,0]
        ]
    }
    return maps.get(choice, maps[1])


# ==============================
# MAIN
# ==============================

if __name__ == "__main__":
    print("Select Map: 1 or 2")
    
    try:
        map_choice = int(input("Enter map number: "))
    except ValueError:
        print("Invalid input! Defaulting to Map 1")
        map_choice = 1

    grid = get_map(map_choice)

    start = (0, 0)
    goal = (len(grid)-1, len(grid[0])-1)

    path = astar(grid, start, goal)

    if path:
        print("Path found:", path)
    else:
        print("No path found")

    visualize(grid, path, start, goal)