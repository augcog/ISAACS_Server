#Imports
import numpy as np
from random import random
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
from collections import deque
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import scipy.io
from mavros_drone import MavrosDrone
import roslibpy
import math
import haversine as hs
import pickle
import sys


def dijkstra(G):
    '''
    Dijkstra algorithm for finding shortest path from start position to end.
    '''
    srcIdx = G.vex2idx[G.startpos]
    dstIdx = G.vex2idx[G.endpos]

    # build dijkstra
    nodes = list(G.neighbors.keys())
    dist = {node: float('inf') for node in nodes}
    prev = {node: None for node in nodes}
    dist[srcIdx] = 0

    while nodes:
        curNode = min(nodes, key=lambda node: dist[node])
        nodes.remove(curNode)
        if dist[curNode] == float('inf'):
            break

        for neighbor, cost in G.neighbors[curNode]:
            newCost = dist[curNode] + cost
            if newCost < dist[neighbor]:
                dist[neighbor] = newCost
                prev[neighbor] = curNode

    # retrieve path
    path = deque()
    curNode = dstIdx
    while prev[curNode] is not None:
        path.appendleft(G.vertices[curNode])
        curNode = prev[curNode]
    path.appendleft(G.vertices[curNode])
    return list(path)

class Line():
    ''' Define line '''
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn /= self.dist # normalize

    def path(self, t):
        return self.p + t * self.dirn


def Intersection(line, center, radius):
    ''' Check line-sphere (circle) intersection '''
    a = np.dot(line.dirn, line.dirn)
    b = 2 * np.dot(line.dirn, line.p - center)
    c = np.dot(line.p - center, line.p - center) - radius * radius

    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return False

    t1 = (-b + np.sqrt(discriminant)) / (2 * a);
    t2 = (-b - np.sqrt(discriminant)) / (2 * a);

    if (t1 < 0 and t2 < 0) or (t1 > line.dist and t2 > line.dist):
        return False

    return True


def distance(obj1, obj2):
    return np.linalg.norm(np.array(obj1) - np.array(obj2))


def isInObstacle(vex, obstacles, radius):
    for obs in obstacles:
        if distance(obs, vex) < radius:
            return True
    return False


def isThruObstacle(line, obstacles, radius):
    for obs in obstacles:
        if Intersection(line, obs, radius):
            return True
    return False


def nearest(G, vex, obstacles, radius):
    Nvex = None
    Nidx = None
    minDist = float("inf")

    for idx, v in enumerate(G.vertices):
        line = Line(v, vex)
        if isThruObstacle(line, obstacles, radius):
            continue

        dist = distance(v, vex)
        if dist < minDist:
            minDist = dist
            Nidx = idx
            Nvex = v

    return Nvex, Nidx

#4D version
def newVertex(randvex, nearvex, stepSize):
    dirn = np.array(randvex) - np.array(nearvex)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min (stepSize, length)

    newvex = (nearvex[0]+dirn[0], nearvex[1]+dirn[1], nearvex[2] + dirn[2], nearvex[3] + dirn[3])
    return newvex

def window(startpos, endpos):
    ''' Define seach window - 2 times of start to end rectangle'''
    width = endpos[0] - startpos[0]
    height = endpos[1] - startpos[1]
    length = endpos[2] - startpos[2]
    winx = startpos[0] - (width / 2.)
    winy = startpos[1] - (height / 2.)
    winz =  startpos[2] - (length / 2.)
    return winx, winy, winz, width, height, length


def isInWindow(pos, winx, winy, winz, width, height, length):
    ''' Restrict new vertex insides search window'''
    if winx < pos[0] < winx+width and \
        winy < pos[1] < winy+height and \
            winz < pos[2] < winz+length:
        return True
    else:
        return False


class Graph:
    ''' Define graph '''
    def __init__(self, startpos, endpos):
        self.startpos = startpos
        self.endpos = endpos

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {startpos:0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}

        self.sx = endpos[0] - startpos[0]
        self.sy = endpos[1] - startpos[1]
        self.sz = endpos[2] - startpos[2]
        self.sw = endpos[3] - startpos[3]

    def add_vex(self, pos):
        try:
            idx = self.vex2idx[pos]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))


    def randomPosition(self):
        rx = random()
        ry = random()
        rz = random()
        rw = random()

        posx = self.startpos[0] - (self.sx / 2.) + rx * self.sx * 2
        posy = self.startpos[1] - (self.sy / 2.) + ry * self.sy * 2
        posz = self.startpos[2] - (self.sz / 2.) + rz * self.sz * 2
        posw = self.startpos[3] - (self.sw / 2.) + rw * self.sw * 2
        return posx, posy, posz, posw

def RRT(startpos, endpos, obstacles, n_iter, radius, stepSize):
    ''' RRT algorithm '''
    G = Graph(startpos, endpos)

    for _ in range(n_iter):
        randvex = G.randomPosition()
        if isInObstacle(randvex, obstacles, radius):
            continue

        nearvex, nearidx = nearest(G, randvex, obstacles, radius)
        if nearvex is None:
            continue

        newvex = newVertex(randvex, nearvex, stepSize)

        newidx = G.add_vex(newvex)
        dist = distance(newvex, nearvex)
        G.add_edge(newidx, nearidx, dist)

        dist = distance(newvex, G.endpos)
        if dist < 2 * radius:
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            G.success = True
            #print('success')
            # break
    return G


def RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize):
    ''' RRT star algorithm '''
    G = Graph(startpos, endpos)

    for i in range(n_iter):
        print(i)
        randvex = G.randomPosition()
        if isInObstacle(randvex, obstacles, radius):
            continue

        nearvex, nearidx = nearest(G, randvex, obstacles, radius)
        if nearvex is None:
            continue

        newvex = newVertex(randvex, nearvex, stepSize)

        newidx = G.add_vex(newvex)
        dist = distance(newvex, nearvex)
        G.add_edge(newidx, nearidx, dist)
        G.distances[newidx] = G.distances[nearidx] + dist

        # update nearby vertices distance (if shorter)
        for vex in G.vertices:
            if vex == newvex:
                continue

            dist = distance(vex, newvex)
            if dist > radius:
                continue

            line = Line(vex, newvex)
            if isThruObstacle(line, obstacles, radius):
                continue

            idx = G.vex2idx[vex]
            if G.distances[newidx] + dist < G.distances[idx]:
                G.add_edge(idx, newidx, dist)
                G.distances[idx] = G.distances[newidx] + dist

        dist = distance(newvex, G.endpos)
        if dist < 2 * radius:
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            try:
                G.distances[endidx] = min(G.distances[endidx], G.distances[newidx]+dist)
            except:
                G.distances[endidx] = G.distances[newidx]+dist

            G.success = True
            #print('success')
            # break
    return G


def plt_sphere(list_center, list_radius, ax):
    for c, r in zip(list_center, list_radius):
        # draw sphere
        u, v = np.mgrid[0:2*np.pi:50j, 0:np.pi:50j]
        x = r*np.cos(u)*np.sin(v)
        y = r*np.sin(u)*np.sin(v)
        z = r*np.cos(v)
        ax.plot_surface(c[0] - x, c[1] - y, c[2] - z, color="red")


def plot(G, obstacles, radius, path=None):
    '''
    Plot RRT, obstacles and shortest path
    '''
    plt.close()
    fig = plt.figure(figsize=(8, 10))
    fig.add_subplot(211)
#     px = [x for x, y, z,w in G.vertices]
#     py = [y for x, y, z,w in G.vertices]
#     pz = [z for x, y, z,w in G.vertices]


    ax = plt.axes(projection='3d')

    plt_sphere([obs[:3] for obs in obstacles], [radius]*len(obstacles), ax)

#     ax.scatter3D(px, py, pz, c='cyan')
    ax.scatter3D(G.startpos[0], G.startpos[1], G.startpos[2], c='black')
    ax.scatter3D(G.endpos[0], G.endpos[1], G.endpos[2], c='black')

#     lines = [(G.vertices[edge[0]][:3], G.vertices[edge[1]][:3]) for edge in G.edges]
#      #lc = mc.LineCollection(lines, colors='green', linewidths=2)
#     lc = Line3DCollection(lines, colors='green', linewidths=2)
#     ax.add_collection(lc)

    if path is not None:
        paths = [(path[i][:3], path[i+1][:3]) for i in range(len(path)-1)]
        #lc2 = mc.LineCollection(paths, colors='blue', linewidths=3)
        lc2 = Line3DCollection(paths, colors='blue', linewidths=3)
        ax.add_collection(lc2)

    #ax.autoscale()
    ax.margins(0.2)

    fig.add_subplot(212)
    plt.plot([w for x, y, z,w in path])
    plt.subplots_adjust(hspace = 5)
    plt.xlabel('timestep')
    plt.ylabel('yaw')
    plt.show()

# Occupancy Grid from MATLAB File
def ObstaclesFromMatLab(matGrid):
    oGrid = []
    for x in range(len(matGrid)):
        for y in range(len(matGrid[0])):
            for z in range(len(matGrid[0][0])):
                for yaw in range(len(matGrid[0][0][0])):
                    if matGrid[x][y][z][yaw] <= 0:
                        oGrid.append((x,y,yaw,z))
    return oGrid

def getLatLong(oldLat, oldLong, dx, dy):
    # Returns the new latitude and longitude from offseting dx and dy meters from the old latitude and longitude
    # Radius of the earth is approximately 6371007 m
    # number of km per degree = ~111km (111.32 in google maps, but range varies
    # between 110.567km at the equator and 111.699km at the poles)
    # 1km in degree = 1 / 111.32km = 0.0089
    # 1m in degree = 0.0089 / 1000 = 0.0000089
    degreeOffset = dy * 0.0000089;
    newLat = oldLat + degreeOffset
    # pi / 180 = 0.018
    newLong = oldLong + degreeOffset / math.cos(oldLat * 0.018)
    trueDist = math.sqrt(dx ** 2 + dy ** 2)
    haversineDist = hs.haversine((oldLat, oldLong), (newLat, newLong)) * 1000
    # if abs(haversineDist - trueDist) > 1:
    #     print(trueDist)
    #     print(haversineDist)
    #     print("Error is greater than 1 meter")
    return newLat, newLong

def path_to_waypoint(startlat, startlong, start, path):
    wp = []
    for node in path:
        newLat, newLong = getLatLong(startlat, startlong, node[0], node[1])

        new_waypoint = {'frame': MavrosDrone.FRAME_REFERENCE.RELATIVE_ALT.value,
                            'command': MavrosDrone.MAV_CMD.NAVIGATE_TO_WAYPOINT.value,
                            'is_current': False, 'autocontinue': True, 'param1': 0,
                            'param2': 0, 'param3': 0, 'param4': node[3], 'x_lat': newLat,
                            'y_long': newLong,
                            'z_alt': node[2]}
        wp.append(new_waypoint)
    return wp


# Main
if __name__ == '__main__':
    num_args = len(sys.argv)
    #to test without running RRT, give any command line argument, and we use wp.txt to test with sample waypoints
    if num_args == 2:
        skip = True
    #client = roslibpy.Ros(host='localhost', port=9090) #uncomment this when testing with ROS
    serviceName = '/mavros/mission/push'
    #client.run() #uncomment this when testing with ROS

    # Load .mat file
    if skip:
        print("**** skipping calculations and using wp.txt")
        file = open('wp.txt', 'rb')
        wp = pickle.load(file)
        service = roslibpy.Service(client, serviceName, 'mavros_msgs/WaypointPush')
        request = roslibpy.ServiceRequest({'waypoints': wp})

        print('Calling /mavros/mission/push service...')
        result = service.call(request)
        print('Service response: {}'.format(result))
        client.terminate()
        print("Done.")

    else: # Do the full RRT calculation using the matlab unsafe set
        mat = scipy.io.loadmat('small.mat')
        matGrid = mat["data"][:,:,:,:,-1]

        # Problem Parameters from Matlab File
        # Might have to change window function
        startpos = (0, 0, 0, 0)
        startlat = 37.91522447196717
        startlong = -122.33786459393546
        endpos = (30,30,5,0)
        obstacles = ObstaclesFromMatLab(matGrid)
        print(obstacles)
        n_iter = 600
        radius = 0.5
        stepSize = 1

        G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)
        # G = RRT(startpos, endpos, obstacles, n_iter, radius, stepSize)

        if G.success:
            path = dijkstra(G)
            print(path)
            #plot(G, obstacles, radius, path)
            wp = path_to_waypoint(startlat, startlong, startpos, path)
            file = open('wp_small.txt', 'wb')
            pickle.dump(wp, file)
            file.close()
            service = roslibpy.Service(client, serviceName, 'mavros_msgs/WaypointPush')
            request = roslibpy.ServiceRequest({'waypoints': wp})

            print('Calling /mavros/mission/push service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
            client.terminate()

        else:
            print("No path")
            #plot(G, obstacles, radius)
