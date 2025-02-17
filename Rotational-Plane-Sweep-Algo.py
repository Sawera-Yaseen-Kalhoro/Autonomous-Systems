from matplotlib import path
import matplotlib.pyplot as plt
import numpy as np
import sys
import math
import pylab as pl

from visgraph_skeleton import Point,Segment

################################################

## functions

def plot_env(vertices, segments, *, showPlot=True):
    """
        Creates a visual representation of the environment based on the vertices and segments.
    """

    # plot the nodes
    for p in vertices:
        plt.plot(p.x,p.y,'ro')
        pl.text(p.x+0.1,p.y+0.1,str(vertices.index(p)),color="black", fontsize=12)

    # plot the edges
    for edge in segments.values():
        p1 = edge.p1
        p2 = edge.p2

        plt.plot([p1.x, p2.x],[p1.y, p2.y], 'r-')
    
    if showPlot:
        plt.show()
    
    return

def calculate_alpha(v, vi):
    """
        Calculate the alpha_i = angle between the horizontal half-line and (v,vi).
    """
    alpha = math.atan2(vi.y - v.y, vi.x - v.x)
    if alpha < 0:
            alpha += 2*np.pi # make the range into [0,2*pi]
    return alpha

def sort_dict(S):
    """
        Sort a dictionary based on the values in an ascending order.
    """
    S_sorted = sorted(S.items(), key=lambda x:x[1])
    S_sorted_dict = dict(S_sorted)
    return S_sorted_dict

def isVisible(v,vi, S):
    """
        Returns True if (v,vi) is visible.
    """
    seg_check = Segment(v,vi)

    # check if v and vi are on the same edge; return True if so
    if are_vertices_on_same_edge(v, vi, seg):
        return True

    # check if v and vi are on the same polygon and the segment is inside the polygon
    if are_vertices_on_same_polygon(v, vi, polygons):
        if is_segment_inside_polygon(v, vi):
            return False
        else:
            return True

    # check if S is empty; return True if so
    if not S:
        return True

    # check if (v,vi) intersects the any edge in the S list; return False if so
    for edge_index in list(S.keys()):
        edge = seg[edge_index]
        if seg_check.intersect(edge)[0] and not (seg_check.intersect(edge)[1] == edge.p1 or seg_check.intersect(edge)[1] == edge.p2):
            return False
    
    return True

def plot_vis_graph(vis_graph):
    for seg_index in vis_graph:
        seg_plot = Segment(vertices[seg_index[0]], vertices[seg_index[1]])
        plt.plot([seg_plot.p1.x, seg_plot.p2.x],[seg_plot.p1.y, seg_plot.p2.y], 'g:')
    
    plt.show()
    return

## helper functions to check for visibility
def are_vertices_on_same_edge(v, vi, seg):
    return any([(v, vi) == (edge.p1, edge.p2) or (vi, v) == (edge.p1, edge.p2) for edge in seg.values()])

def are_vertices_on_same_polygon(v, vi, polygons):
    return any([vertices.index(v) in polygon and vertices.index(vi) in polygon for polygon in polygons])

def is_segment_inside_polygon(v, vi):
    ## assuming (v,vi) is in a polygon, if it's not part of any segment, then it's inside the polygon
    return Segment(v,vi) not in seg.values()


################################################

## main execution

# import the environment in a csv file
file_name = sys.argv[1]
arr = np.loadtxt(file_name, delimiter=',')

# convert every points into Point class
vertices = [] # initiate list of points

for point in arr:
    vertices.append(Point(point[1],point[2]))

# split points into the corresponding polygons
polygons = [] # initiate a list of polygons
curr_pol = [] # initiate a list of points in a polygon
curr_pol_index = 0 # initiate polygon index

for point_index in range(len(arr)):
    if int(arr[point_index,0]) != curr_pol_index:
        polygons.append(curr_pol)
        curr_pol = []
        curr_pol_index = arr[point_index,0]
        
    curr_pol.append(point_index)
polygons.append(curr_pol)

# convert every edge into Segment class
seg = {} # create a dictionary "seg" which contains the vertices forming the edges and the Segment objects
for polygon in polygons:
    if len(polygon) > 1:
        # update the list (dict) of edges
        seg.update({(i,i+1): Segment(vertices[i],vertices[i+1]) for i in polygon if i != polygon[-1]})
        # connect the last and the first points in a polygon
        seg[(polygon[-1],polygon[0])] = Segment(vertices[polygon[0]],vertices[polygon[-1]])

# plot the environment
plot_env(vertices,seg, showPlot=False)

# start of RPS
# initialize visibility graph
vis_graph = []

# loop for all vertices except the goal point
for o in range(len(vertices)-1):
    print("Visibility from vertex: ",o)
    v = vertices[o] # assign the vertex to be checked

    # Generate alpha list
    alpha = {} # initiate alpha as a dictionary containing the indices of the vertices and the angles

    for vertex in vertices:
        # skip if the vertex is the same as the vertex checked
        if vertex == v:
            continue

        # calculate the angle alpha_i
        alpha_i = calculate_alpha(v,vertex)
        alpha[vertices.index(vertex)] = alpha_i
    
    print("alpha = ",alpha)
    
    # sort the alpha to create the epsilon list
    alpha_sorted = sort_dict(alpha)
    epsilon = list(alpha_sorted.keys())
    
    print("Vertex sweep order: ",epsilon)

    # initialize S list
    S = {} # initiate S as a dictionary containing the vertices forming the edges and the distance to v

    # initialize horizontal half-line
    hor = Segment.point_angle_length(v,0,10000)
    # assign intersecting edges into the S list (dict)
    for seg_index,seg_object in seg.items():
        if hor.intersect(seg_object)[0]:
            S[seg_index] = v.dist_line(seg_object)
            S = sort_dict(S)
    
    print("Initial S :", S, "\n")

    # sweep for all vertices in the order of epsilon
    for i in epsilon:
        vi = vertices[i]
        print("Check vertex ",i)

        # check if (v,vi) is visible; if yes, add it to the visibility graph
        if isVisible(v,vi, S):
            if (i,o) in vis_graph: # if the edge already exists in the vis graph, don't add it
                print(f"{(o,i)} is already in the visibility graph.")
            else:
                vis_graph.append((o,i))
                print(f"Add {(o,i)} to the visibility graph.")
        
        # update S list (dict)
        for edge_index,edge_object in seg.items(): # check all edges
            if (edge_object.p1 == vi) or (edge_object.p2 == vi): # if vi belongs to the edge
                if edge_index not in S.keys(): # if the edge is not in S
                    # add it into S list (dict)
                    S[edge_index] = v.dist_line(edge_object)
                    print(f"Add edge {edge_index} to the S list.")
                else: # if the edge is in S
                    # remove it from S list (dict)
                    S.pop(edge_index)
                    print(f"Remove edge {edge_index} from the S list.")
        
        # re-sort S list (dict)
        S = sort_dict(S)
        print("Updated S :", S, "\n")
    
    print("===============================")

print("Visibility graph: ",vis_graph)

# visualize the visibility graph
plot_vis_graph(vis_graph)
