import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import heapq
from scipy.interpolate import CubicSpline

grid_size = 10
grid = np.zeros((grid_size, grid_size))

start = (0, 0)
goal = (9, 9)

# Obstacles
grid[3, 5] = -1
grid[6, 2:8] = -1
grid[1:5, 9] = -1
grid[1:8, 3] = -1

# Color map
cmap = ListedColormap([
    'black', 'white', 'green', 'red',
    'purple', 'blue', 'yellow', 'cyan'
])

# -----------------------------
# Heuristic (Euclidean)
# -----------------------------
def heuristic(a, b):
    return ((a[0] - b[0])**2 + (a[1] - b[1])**2) ** 0.5

# -----------------------------
# Neighbors (8-direction + corner protection)
# -----------------------------
def get_neighbors(pos):
    x, y = pos
    moves = [
        (1,0), (-1,0), (0,1), (0,-1),
        (1,1), (1,-1), (-1,1), (-1,-1)
    ]

    result = []
    for dx, dy in moves:
        nx, ny = x + dx, y + dy

        if 0 <= nx < grid_size and 0 <= ny < grid_size:
            if grid[nx, ny] == -1:
                continue

            # prevent cutting corners
            if dx != 0 and dy != 0:
                if grid[x + dx, y] == -1 or grid[x, y + dy] == -1:
                    continue

            result.append((nx, ny))

    return result

# -----------------------------
# Line of sight (for smoothing)
# -----------------------------
def line_of_sight(p1, p2):
    x1, y1 = p1
    x2, y2 = p2

    dx = x2 - x1
    dy = y2 - y1

    steps = max(abs(dx), abs(dy))

    for i in range(steps + 1):
        t = i / steps if steps != 0 else 0
        x = int(round(x1 + dx * t))
        y = int(round(y1 + dy * t))

        if grid[x, y] == -1:
            return False

    return True

# -----------------------------
# Path smoothing
# -----------------------------
def smooth_path(path):
    if not path:
        return path

    smoothed = [path[0]]
    i = 0

    while i < len(path) - 1:
        j = len(path) - 1

        while j > i:
            if line_of_sight(path[i], path[j]):
                smoothed.append(path[j])
                i = j
                break
            j -= 1

    return smoothed

# -----------------------------
# Cubic spline curve
# -----------------------------
def create_spline(path):
    if len(path) < 3:
        return path

    path = np.array(path)

    t = np.arange(len(path))
    x = path[:, 0]
    y = path[:, 1]

    cs_x = CubicSpline(t, x)
    cs_y = CubicSpline(t, y)

    t_fine = np.linspace(0, len(path) - 1, 200)

    x_smooth = cs_x(t_fine)
    y_smooth = cs_y(t_fine)

    return list(zip(x_smooth, y_smooth))

# -----------------------------
# Draw grid (A* exploration)
# -----------------------------
def draw(open_set, closed_set, path=None):
    display = np.ones_like(grid)

    display[grid == -1] = 0

    for node in open_set:
        display[node] = 5

    for node in closed_set:
        display[node] = 6

    if path:
        for p in path:
            display[p] = 4

    display[start] = 2
    display[goal] = 3

    plt.clf()
    plt.imshow(display, cmap=cmap)

    plt.xticks(np.arange(-0.5, grid_size, 1))
    plt.yticks(np.arange(-0.5, grid_size, 1))
    plt.grid(True)
    plt.tick_params(left=False, bottom=False,
                    labelleft=False, labelbottom=False)

    plt.pause(0.15)

# -----------------------------
# Draw spline curve
# -----------------------------
def draw_curve(curve):
    display = np.ones_like(grid)
    display[grid == -1] = 0

    plt.clf()
    plt.imshow(display, cmap=cmap)

    xs = [p[1] for p in curve]
    ys = [p[0] for p in curve]

    plt.plot(xs, ys)
    plt.scatter(start[1], start[0])
    plt.scatter(goal[1], goal[0])

    plt.xticks(np.arange(-0.5, grid_size, 1))
    plt.yticks(np.arange(-0.5, grid_size, 1))
    plt.grid(True)
    plt.tick_params(left=False, bottom=False,
                    labelleft=False, labelbottom=False)

    plt.pause(0.01)

# -----------------------------
# A* algorithm
# -----------------------------
def astar_visual():
    open_heap = []
    heapq.heappush(open_heap, (0, start))

    came_from = {}
    g_score = {start: 0}

    open_set = {start}
    closed_set = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)

        open_set.discard(current)
        closed_set.add(current)

        draw(open_set, closed_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        for neighbor in get_neighbors(current):

            dx = abs(neighbor[0] - current[0])
            dy = abs(neighbor[1] - current[1])

            move_cost = 1.414 if dx == 1 and dy == 1 else 1
            temp_g = g_score[current] + move_cost

            if neighbor in closed_set:
                continue

            if neighbor not in g_score or temp_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g

                f = temp_g + heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f, neighbor))
                open_set.add(neighbor)

    return None

# -----------------------------
# RUN
# -----------------------------
plt.figure(figsize=(6,6))

path = astar_visual()

if path:
    # Show original path
    for i in range(len(path)):
        draw(set(), set(), path[:i+1])

    plt.pause(1)

    # Smooth path
    smooth = smooth_path(path)

    # Create spline
    curve = create_spline(smooth)

    # Animate curve
    for i in range(len(curve)):
        draw_curve(curve[:i+1])

    print("✅ Curved trajectory generated!")

else:
    print("❌ No path found!")

plt.show()