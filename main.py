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
def heuristic(a, b, use_heuristic=True):
    if not use_heuristic:
        return 0  # Dijkstra mode
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


# =========================
# NEIGHBORS (8-direction)
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
# A* / DIJKSTRA
# =========================
def a_star(grid, start, goal, use_heuristic=True):
    open_list = []
    closed = set()

    start_node = Node(start)
    heapq.heappush(open_list, start_node)

    nodes_explored = 0

    while open_list:
        current = heapq.heappop(open_list)
        nodes_explored += 1

        if current.position == goal:
            path = []
            while current:
                path.append(current.position)
                current = current.parent
            return path[::-1], nodes_explored

        closed.add(current.position)

        for pos in get_neighbors(current, grid):
            if pos in closed:
                continue

            node = Node(pos, current)

            node.g = current.g + 1
            node.h = heuristic(pos, goal, use_heuristic)
            node.f = node.g + node.h

            heapq.heappush(open_list, node)

    return None, nodes_explored


# =========================
# VISUALIZATION (SIDE-BY-SIDE)
# =========================
def visualize(grid, path_astar, path_dijkstra, start, goal):
    grid_array = np.array(grid)

    fig, axs = plt.subplots(1, 2, figsize=(12,6))

    titles = ["A* (Heuristic)", "Dijkstra (h = 0)"]
    paths = [path_astar, path_dijkstra]

    for ax, path, title in zip(axs, paths, titles):
        ax.imshow(grid_array)

        # Obstacles
        for r in range(len(grid)):
            for c in range(len(grid[0])):
                if grid[r][c] == 1:
                    ax.add_patch(
                        plt.Rectangle((c-0.5, r-0.5), 1, 1, color='black')
                    )

        # Path
        if path:
            x = [p[1] for p in path]
            y = [p[0] for p in path]
            ax.plot(x, y, linewidth=3)

        # Start & Goal
        ax.scatter(start[1], start[0], s=100)
        ax.scatter(goal[1], goal[0], s=100, marker='X')

        ax.set_title(title)
        ax.invert_yaxis()

    plt.savefig("comparison.png", bbox_inches='tight')
    plt.show()


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

    start = (0,0)
    goal = (5,5)

    # A*
    path_astar, nodes_a = a_star(grid, start, goal, True)

    # Dijkstra
    path_dijkstra, nodes_d = a_star(grid, start, goal, False)

    print("A* Path Length:", len(path_astar))
    print("Dijkstra Path Length:", len(path_dijkstra))

    print("A* Nodes Explored:", nodes_a)
    print("Dijkstra Nodes Explored:", nodes_d)

    visualize(grid, path_astar, path_dijkstra, start, goal)