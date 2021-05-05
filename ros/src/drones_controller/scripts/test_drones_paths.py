#!/usr/bin/env python

import sys
import time

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from drones_controller.msg import DroneWaypoint
from drones_controller.msg import DronePath

import matplotlib.pyplot as plt

from math import sqrt

from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import box
from shapely.geometry import LineString
from shapely.geometry import MultiLineString

from networkx import Graph
from networkx import shortest_path
from networkx import minimum_spanning_tree

class Route:
    def __init__(self, polygon, diameter):
        self.set_cell_size(diameter)
        self.set_bounds(polygon)
        self.compute_num_of_cells()
        self.set_frame()
        self.create_centers()
        self.create_nodes()
        self.create_walls()
        self.pick_edges()
        self.create_circular_route()

    def set_cell_size(self, diameter):
        self.cell_size = sqrt((diameter**2)/2)
        self.half_cell_size = self.cell_size / 2
        self.double_cell_size = self.cell_size * 2

    def set_bounds(self, polygon):
        self.polygon = polygon
        self.minx = polygon.bounds[0]
        self.miny = polygon.bounds[1]
        self.maxx = polygon.bounds[2]
        self.maxy = polygon.bounds[3]
        self.width = self.maxx - self.minx
        self.height = self.maxy - self.miny

    def compute_num_of_cells(self):
        self.num_cell_width = self.width / self.cell_size
        self.num_cell_height = self.height / self.cell_size
        self.num_double_cell_width = self.width / self.double_cell_size
        self.num_double_cell_height = self.height / self.double_cell_size

    def set_frame(self):
        wall_top = LineString(((self.minx,self.maxy),(self.maxx,self.maxy)))
        wall_left = LineString(((self.minx,self.miny),(self.minx,self.maxy)))
        wall_right = LineString(((self.maxx,self.miny),(self.maxx,self.maxy)))
        wall_bottom = LineString(((self.minx,self.miny),(self.maxx,self.miny)))
        self.frame = MultiLineString([wall_top, wall_left, wall_right, wall_bottom])

    #TODO: merge to one function?
    def create_centers(self):
        self.centers = []
        for i in range(int(self.num_cell_width)):
            for j in range(int(self.num_cell_height)):
                center = (self.minx + self.half_cell_size + i * self.cell_size, self.miny + self.half_cell_size + j * self.cell_size)
                if self.polygon.contains(Point(center)):
                    self.centers.append(center)
    def create_nodes(self):
        self.nodes = []
        for i in range(int(self.num_double_cell_width)):
            for j in range(int(self.num_double_cell_height)):
                node = (self.minx + self.cell_size + i * self.double_cell_size, self.miny + self.cell_size + j * self.double_cell_size)
                if self.polygon.contains(Point(node)):
                    self.nodes.append(node)

    def create_walls(self):
        self.g = create_square_lattice_graph(self.nodes, self.double_cell_size)
        t = minimum_spanning_tree(self.g, algorithm='kruskal')
        # t = minimum_spanning_tree(self.g, algorithm='prim')
        self.vertical_edges, self.horizontal_edges = split_edges(t.edges)
        self.walls = MultiLineString(self.vertical_edges + self.horizontal_edges)

    def pick_edges(self):
        surrounding_edges = set()
        for edge in self.horizontal_edges:
            surrounding_edges.update(surrounding_edges_for_horizontal_edge(edge, self.half_cell_size))
        for edge in self.vertical_edges:
            surrounding_edges.update(surrounding_edges_for_vertical_edge(edge, self.half_cell_size))
        self.route_edges = set()
        for se in surrounding_edges:
            line = LineString(se)
            if not line.crosses(self.walls):
                self.route_edges.add(se)
        
    def create_circular_route(self):
        g = Graph()
        g.add_edges_from(self.route_edges)
        node = list(g.nodes)[0]
        neighbors  = list(g.adj[node])
        g.remove_node(node)
        self.path = shortest_path(g, neighbors[0], neighbors[1])
        self.path.append(node)
        # and now the path is a circle

    def visualize(self):
        fig, axs = plt.subplots()
        axs.set_aspect('equal', 'datalim')
        for geom in self.frame:
            xs, ys = geom.xy
            axs.fill(xs, ys, alpha=1, fc='r', ec='black')
        for geom in self.walls:
            xs, ys = geom.xy
            axs.fill(xs, ys, alpha=1, fc='r', ec='black')
        for geom in self.centers:    
            xs, ys = geom    
            axs.plot(xs, ys, 'bo')
        for geom in self.nodes:  
            xs, ys = geom    
            axs.plot(xs, ys, 'y+')
        for geom in self.route_edges:
            geom = LineString(geom)
            xs, ys = geom.xy
            axs.fill(xs, ys, alpha=1, fc='r', ec='red')
        plt.show()

    def __str__():
        lines = []
        str.append('cell size: {0}*{0}'.format(cell_size))
        str.append('cell diameter: {}'.format(diameter))
        str.append('minx: {}'.format(minx))
        str.append('miny: {}'.format(miny))
        str.append('maxx: {}'.format(maxx))
        str.append('maxy: {}'.format(maxy))
        str.append('width: {}'.format(width))
        str.append('height: {}'.format(height))
        str.append('num_cell_width: {}'.format(num_cell_width))
        str.append('num_cell_height: {}'.format(num_cell_height))
        return '\n'.join(lines) 

def addPointToPath(pathArr, x, y, z, minExecutionTick = 0):
    droneWaypoint = DroneWaypoint()
    goal = PoseStamped()
    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map" #TODO
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0
    droneWaypoint.goalPose = goal
    droneWaypoint.minExecutionTick = minExecutionTick
    pathArr.append(droneWaypoint)


def create_neighbors_diag(center, distance):
    tr = (center[0] + distance, center[1] + distance)
    br = (center[0] + distance, center[1] - distance)
    tl = (center[0] - distance, center[1] + distance)
    bl = (center[0] - distance, center[1] - distance)
    return [tl, bl, tr, br]

def split_edges(edges):
    edges = [sorted(edge) for edge in edges]
    vertical_edges = []
    horizontal_edges = []
    for edge in edges:
        if edge[0][0] == edge[1][0]:
            vertical_edges.append(edge)
        elif edge[0][1] == edge[1][1]:
            horizontal_edges.append(edge)
        else:
            raise Exception(edge)
    return sorted(vertical_edges, key=lambda edge: edge[0]), sorted(horizontal_edges, key=lambda edge: tuple(reversed(edge[0])))


def create_square_lattice_graph(nodes, double_cell_size):
    g = Graph()
    for s in nodes:
        t1 = (s[0] + double_cell_size, s[1])
        t2 = (s[0] - double_cell_size, s[1])
        t3 = (s[0], s[1] + double_cell_size)
        t4 = (s[0], s[1] - double_cell_size)
        for t in [t1, t2, t3, t4]:
            if t in nodes:
                g.add_edge(s, t)
    return g

def surrounding_edges_for_horizontal_edge(edge, half_cell_size):
    se = []
    left_neighbors = create_neighbors_diag(edge[0], half_cell_size)
    right_neighbors = create_neighbors_diag(edge[1], half_cell_size)
    se.append((left_neighbors[0], left_neighbors[2]))
    se.append((left_neighbors[1], left_neighbors[3]))
    se.append((left_neighbors[1], left_neighbors[0]))
    se.append((right_neighbors[0], right_neighbors[2]))
    se.append((right_neighbors[1], right_neighbors[3]))
    se.append((right_neighbors[3], right_neighbors[2]))
    se.append((left_neighbors[2], right_neighbors[0]))
    se.append((left_neighbors[3], right_neighbors[1]))
    return se

def surrounding_edges_for_vertical_edge(edge, half_cell_size):
    se = []
    bottom_neighbors = create_neighbors_diag(edge[0], half_cell_size)
    top_neighbors = create_neighbors_diag(edge[1], half_cell_size)
    se.append((bottom_neighbors[1], bottom_neighbors[0]))
    se.append((bottom_neighbors[3], bottom_neighbors[2]))
    se.append((bottom_neighbors[1], bottom_neighbors[3]))
    se.append((top_neighbors[1], top_neighbors[0]))
    se.append((top_neighbors[3], top_neighbors[2]))
    se.append((top_neighbors[0], top_neighbors[2]))
    se.append((bottom_neighbors[0], top_neighbors[1]))
    se.append((bottom_neighbors[2], top_neighbors[3]))
    return se

def rotate(l, n):
    return l[n:] + l[:n]

def to_k_drones(path, k):
    if k > len(path):
        raise Exception('the number of drones is larger then the number of cells')
    paths = []
    for i in range(k):
        paths.append(path)
        path = rotate(path, len(path)//k)
    return paths

def add_times(paths, interval):
    paths_new = []
    for path in paths:
        paths_new.append(list(map(lambda p: (p[0], p[1], p[2], path.index(p)*interval), path)))
    return paths_new

def algo_1(polygon, k, h, d, t):
    route = Route(polygon, d)
    # route.visualize()
    path = list(map(lambda p: (p[0], p[1], h), route.path))
    paths = to_k_drones(path, k)
    paths = add_times(paths,t)
    return paths

# def algo_2(polygon, k, h, d, t):
#     path = Route(polygon, d)
#     path = list(map(lambda p: (p[0], p[1], h), path))
#     paths = to_k_drones(path, k)
#     return paths

def ros_commands(paths):
    # droneId = 0
    # armPub = rospy.Publisher('/airsim_node/Drone'+str(droneId)+'/cmd/arm', Empty, queue_size=10)
    # offboardModePub = rospy.Publisher('/uav'+str(droneId)+'/safety/cmd/set_mode', String, queue_size=10)
    drone_str = '/airsim_node/Drone{}/set_path'
    rospy.init_node('test_drones_paths')
    publishers = []
    messages = []
    for i in range(len(paths)):
        publishers.append(rospy.Publisher(drone_str.format(i+1), DronePath, queue_size=10))
        message = DronePath()
        message.waypoints = []
        for point in paths[i]:
            addPointToPath(message.waypoints, point[0], point[1], point[2] + 10*i, point[3])
        message.waypointWaitSeconds = 0.0
        message.isPathCyclic = True
        messages.append(message)
    for publisher, msg in zip(publishers, messages):
        time.sleep(1.0)
        publisher.publish(msg)
    print('published')
    sys.exit()

polygon = Polygon([(0,30),(30,70),(70,80),(80,20),(60,0)])
rectangle = Polygon([(0.5,0.5),(20.5,0.5),(20.5,2.5),(0.5,2.5),(0.5,0.5)])
rhombus_big = Polygon([(-100,0),(0,100),(100,0),(0,-100),(-100,0)])
rhombus_small_1 = Polygon([(-10,0),(0,10),(10,0),(0,-10),(-10,0)])
rhombus_small_2 = Polygon([(-10,0),(0,9),(9,0),(0,-10),(-10,0)])
rhombus_small_3 = Polygon([(-10,0),(0,9),(10,0),(0,-10),(-10,0)])
rhombus_small_4 = Polygon([(-10,0),(0,10),(9,0),(0,-10),(-10,0)])
square_big = Polygon([(-100,-100),(-100,100),(100,100),(100,-100),(-100,-100)])
square_small_1 = Polygon([(-10,-10),(-10,10),(10,10),(10,-10),(-10,-10)])
square_small_2 = Polygon([(-10,-10),(-10,9),(9,9),(9,-10),(-10,-10)])
square_small_3 = Polygon([(-10,-10),(-10,9),(10,9),(10,-10),(-10,-10)])
square_small_4 = Polygon([(-10,-10),(-10,10),(9,10),(9,-10),(-10,-10)])

path1 = [(0, 0, -10, 2), (0, 3, -10, 2), (3, 3, -10, 2), (3, 0, -10, 2)]
path2 = [(0, 3, -10, 2), (3, 3, -10, 2), (3, 0, -10, 2), (0, 0, -10, 2)]
path3 = [(3, 3, -10, 2), (3, 0, -10, 2), (0, 0, -10, 2), (0, 3, -10, 2)]
path4 = [(3, 0, -10, 2), (0, 0, -10, 2), (0, 3, -10, 2), (3, 3, -10, 2)]

paths = [path1,path2,path3,path4]
# paths = algo_1(square_small_1, 4, -10, 5*sqrt(2), 1)


for path in paths:
    print(path)

ros_commands(paths)