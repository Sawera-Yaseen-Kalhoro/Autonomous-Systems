# Importing Libraries
import math
import random
import numpy as np
from matplotlib import pyplot as plt
from PIL import Image

# __________________________________________________

# Copied from the Lab guide

# Load grid map
image = Image.open("map0.png").convert("L")
grid_map = np.array(image.getdata()).reshape(image.size[0], image.size[1]) / 255
# binarize the image
grid_map[grid_map > 0.5] = 1
grid_map[grid_map <= 0.5] = 0
# Invert colors to make 0 -> free and 1 -> occupied
grid_map = (grid_map * -1) + 1
# Show grid map
plt.matshow(grid_map)
plt.colorbar()
plt.show()

# __________________________________________________


class Node:
    def __init__(self, q):
        self.q = np.array(q)
        self.parent = None


class Graph:
    def __init__(self, qstart):
        self.vertices = [qstart]
        self.edges = []

    def add_vertex(self, qnew):
        self.vertices.append(qnew)

    def add_edge(self, qnear, qnew):
        self.edges.append((qnear, qnew))
        qnew.parent = qnear


# ______________________________________________
# Creating Functions

def RRT(C, K, delta_q, p, qstart, qgoal):
    # RRT Algorithm
    G = Graph(qstart)

    for k in range(K):
        qrand = RAND_CONF(C, p, qgoal)
        qnear = NEAREST_VERTEX(qrand, G)
        qnew = NEW_CONF(qnear, qrand, delta_q)

        if is_segment_free(qnear.q, qnew.q, C):
            G.add_vertex(qnew)
            G.add_edge(qnear, qnew)

            if np.array_equal(qnew.q, qgoal.q):
                filled_path = FILL_PATH(G)
                print(f"Path found in {k} iterations")
                print(f"Distance: {calculate_path_distance(filled_path[1]):.6f}")
                print("PATH to follow:")
                return filled_path

    return "No solution found"


def RAND_CONF(C, p, qgoal):
    # Generate a new configuration randomly
    if random.uniform(0, 1) < p:
        return Node(qgoal.q)
    else:
        # image = Image.open(C).convert("L")
        return Node([random.uniform(0, image.size[0]), random.uniform(0, image.size[1])])


def distance(q1, q2):
    # Euclidean Distance
    return np.linalg.norm(np.array(q1.q) - np.array(q2.q))


def NEAREST_VERTEX(qrand, G):
    # Find the nearest vertex
    return min(G.vertices, key=lambda node: distance(node, qrand))

def calculate_path_distance(path):
    distance = 0
    for i in range(len(path) - 1):
        distance += np.linalg.norm(np.array(path[i]) - np.array(path[i + 1]))
    return distance


def NEW_CONF(qnear, qrand, delta_q):
    # Generate a new configuration qnew based on qnear qrand
    distance_qnear_qrand = distance(qnear, qrand)

    if distance_qnear_qrand < delta_q:
        return Node(qrand.q)
    else:
        direction = np.array(np.array(qrand.q) - np.array(qnear.q), dtype=np.float64)
        direction /= np.linalg.norm(direction)
        new_q = np.array(qnear.q) + delta_q * direction
        return Node(new_q.round().astype(int))


def is_segment_free(qnear, qnew, C):
    # Check if the segment between qnear and qnew crosses only free space
    steps = 10

    x_values = np.linspace(qnear[0], qnew[0], steps)
    y_values = np.linspace(qnear[1], qnew[1], steps)

    for x, y in zip(x_values, y_values):
        x, y = int(x), int(y)
        if 0 <= x < len(C) and 0 <= y < len(C[0]) and C[x][y] == 0:
            continue
        else:
            return False

    return True

def SMOOTH_PATH(path, C):
    current = path[-1]
    smooth_path = [current]

    while not np.array_equal(current, path[0]):    
        for p in path:
            if is_segment_free(current, p, C):
                current = p
                break
        smooth_path.append(current)

    return smooth_path


def FILL_PATH(G):
    # Reconstruct the path from the goal to the start using the graph G
    path = []
    current_node = G.vertices[-1]

    while current_node:
        path.insert(0, current_node.q)

        current_node = current_node.parent
    return G, path

def plot_path(C, G, filled_path, smoothed_path=None):
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # Plotting original path
    axes[0].matshow(C)
    vertices = np.array([node.q for node in G.vertices])
    axes[0].scatter(vertices[:, 1], vertices[:, 0], c='red')
    for edge in G.edges:
        axes[0].plot([edge[0].q[1], edge[1].q[1]], [edge[0].q[0], edge[1].q[0]], 'blue')
    path = np.array(filled_path[1])
    axes[0].plot(path[:, 1], path[:, 0], 'green')
    axes[0].set_title('Original Path')

    # Plotting smoothed path
    if smoothed_path:
        axes[1].matshow(C)
        axes[1].scatter(vertices[:, 1], vertices[:, 0], c='red')
        for edge in G.edges:
            axes[1].plot([edge[0].q[1], edge[1].q[1]], [edge[0].q[0], edge[1].q[0]], 'blue')
        smoothed_path = np.array(smoothed_path[1])
        axes[1].plot(smoothed_path[:, 1], smoothed_path[:, 0], 'orange', label='Smoothed Path')
        axes[1].set_title('Smoothed Path')

    plt.legend()
    plt.show()

# ______________________________________________
# Main Execution
C = grid_map
K = 10000
delta_q = 10
p = 0.2
qstart = Node([10, 10])
qgoal = Node([90, 70])

result = RRT(C, K, delta_q, p, qstart, qgoal)

if isinstance(result, tuple):  
    G, path = result
    smoothed_path = SMOOTH_PATH(path, C)

    print("Original Path:")
    for node in path:
        print(tuple(node))

    print("\nSmoothed Path:")
    for node in smoothed_path:
        print(tuple(node))

    # Plotting both paths
    plot_path(C, G, (G, path), smoothed_path=(G, smoothed_path))
else:
    print("No path")
