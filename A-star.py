import csv
from matplotlib import pyplot as plt
import numpy as np
import math
from typing import List

#####################################
# copied from loadcsv.py

class Vertex:
    """
    Vertex class defined by x and y coordinate.
    """
    # constructor or initializer of vertex class

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def dist(self, p: "Vertex"):
        """
        Return distance between vertices
         Parameters:
            p: input vertex to calculate distance to.
         Returns:
            Distance to vertex from this vertex object
        """

        return math.sqrt((self.x - p.x)**2 + (self.y - p.y)**2)

    # method to define print() function of object vertex
    def __str__(self):
        return "({}, {})".format(np.round(self.x, 2), np.round(self.y, 2))

    # method to define print() function of list[] of object vertex
    def __repr__(self):
        return "({}, {})".format(np.round(self.x, 2), np.round(self.y, 2))

def plot(vertices, edges):

    for v in vertices:
        plt.plot(v.x, v.y, 'r+')

    for e in edges:
        plt.plot([vertices[e[0]].x, vertices[e[1]].x],
                 [vertices[e[0]].y, vertices[e[1]].y],
                 "g--")

    for i, v in enumerate(vertices):
        plt.text(v.x + 0.2, v.y, str(i))
    plt.axis('equal')


def load_vertices_from_file(filename: str):
    # list of vertices
    vertices: List[Vertex] = []
    with open(filename, newline='\n') as csvfile:
        v_data = csv.reader(csvfile, delimiter=",")
        next(v_data)
        for row in v_data:
            vertex = Vertex(float(row[1]), float(row[2]))
            vertices.append(vertex)
    return vertices


def load_edges_from_file(filename: str):
    edges = []
    with open(filename, newline='\n') as csvfile:
        reader = csv.reader(csvfile, delimiter=",")
        next(reader)
        for row in reader:
            edges.append((int(row[0]), int(row[1])))
    return edges


print("vertices from file")
# save each "vertex" in vertices list
vertices = load_vertices_from_file("env_0.csv")
for elem in vertices:
    print(elem)

print("\ndistance from vertex 0 to vertex 1")
print(vertices[0].dist(vertices[1]))

print("\nedges from file")
# save each edge in "edges" list
edges = load_edges_from_file("visibility_graph_env_0.csv")
for elem in edges:
    print(elem)

plot(vertices, edges)
# plt.show()
plt.axis('equal')

###################################################

# FUNCTIONS

def heuristic_fcn(vertices):
    """
        Computes the heuristic function = Euclidean distance of each node to the goal.
    """

    h = []

    for i in range(len(vertices)-1):
        h.append(np.sqrt((vertices[i].x - vertices[-1].x)**2 + (vertices[i].y - vertices[-1].y)**2))
    
    return h

def reconstruct_path(cameFrom, current):
    total_path = [current]
    while current in cameFrom.keys():
        current = cameFrom[current]
        total_path.insert(0, current)
    return total_path


def distance(vertex1, vertex2):
    return math.sqrt((vertex1.x - vertex2.x) ** 2 + (vertex1.y - vertex2.y) ** 2)


def get_neighbors(node, edges):
    neighbors = []
    for edge in edges:
        if edge[0] == node:
            neighbors.append(edge[1])
        elif edge[1] == node:
            neighbors.append(edge[0])
    return neighbors


def A_star(start, goal, h):
    """
        Runs the A* algorithm to find the path from a vis graph.
        start : index of the start point (usually 0)
        goal: index of the goal point
        h: heuristic function (i.e. Euclidean distance of each node to the goal)
    """

    # initialize open set
    openSet = [start]

    # initialize a list cameFrom where cameFrom[n] = the node preceding the node n, with the lowest cost
    cameFrom = []

    # initialize gScore where gScore[n] = cost of the cheapest path from start to n
    gScore = [10000] * len(h) # initialize with a very big value
    gScore[start] = 0

    # initialize fScore
    fScore = {vertex: float('inf') for vertex in range(len(vertices))}
    fScore[start] = gScore[start] + h[start]


    while openSet:
        current = min(openSet, key=lambda x: fScore[x])

        if current == goal:
            return reconstruct_path(cameFrom, current)

        openSet.remove(current)

        for neighbor in get_neighbors(current, edges):
            tentative_gScore = gScore[current] + distance(vertices[current], vertices[neighbor])

            if tentative_gScore < gScore[neighbor]:
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = gScore[neighbor] + h[neighbor]

                if neighbor not in openSet:
                    openSet.add(neighbor)

    return None




###################################################

# MAIN EXECUTION

# vertices = list containing the (x,y) of each node
# edges = list containing (index p1, index p2) forming each segment in the vis graph

# compute heuristic function
h = heuristic_fcn(vertices)


# Load vertices and edges
vertices = load_vertices_from_file("env_0.csv")
edges = load_edges_from_file("visibility_graph_env_0.csv")

# Define start and goal vertices
start_vertex = vertices[0]
goal_vertex = vertices[-1]

# Run A* algorithm
path = A_star(0, len(vertices) - 1, vertices, edges, h)

if path:
    print("Optimal Path:", path)
    total_cost = sum(vertices[path[i]].dist(vertices[path[i + 1]]) for i in range(len(path) - 1))
    print("Total Cost:", total_cost)

    # Plot the result
    plot(vertices, edges)
    plt.plot([vertices[i].x for i in path], [vertices[i].y for i in path], marker='o', color='b')
    plt.show()
else:
    print("No path found.")