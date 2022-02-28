# -*- coding: utf-8 -*-
"""RRT_Star.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/10WGgNBBcAL8IFvkLCN4Kl8KVEdCtsyCM
"""

'''
MIT License
Copyright (c) 2019 Fanjin Zeng
This work is licensed under the terms of the MIT license, see <https://opensource.org/licenses/MIT>.
https://gist.github.com/Fnjn/58e5eaa27a3dc004c3526ea82a92de80
RRT = rapidly exploring random tree
'''

import numpy as np
from random import random
from collections import deque
from mavros_drone import MavrosDrone
import ipympl
import haversine as hs
import scipy.io
import pickle
import math

def RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize, matGrid):
    ''' RRT star algorithm '''
    G = Graph(startpos, endpos)

    for i in range(n_iter):
        print(i)
        randvex = G.randomPosition()
        if isInObstacle(randvex, obstacles, radius):
            continue

        nearvex, nearidx = nearest(G, randvex, obstacles, radius, matGrid)
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
            if isThruObstacleDiscretized(line, matGrid):
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
            print('success')
            break
    return G

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
        self.sw = endpos[3] - startpos[3] #added w

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

        posx = self.startpos[0] + rx * self.sx
        posy = self.startpos[1] + ry * self.sy
        posz = self.startpos[2] + rz * self.sz
        posw = self.startpos[3] + rw * self.sw
        return posx, posy, posz, posw

class Line():
    ''' Define line '''
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn /= self.dist # normalize

    def path(self, t):
        return self.p + t * self.dirn

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

def distance(obj1, obj2):
    return np.linalg.norm(np.array(obj1) - np.array(obj2))

def isInObstacle(vex, obstacles, radius):
    for obs in obstacles:
        if distance(obs, vex) < radius:
            return True
    return False

def nearest(G, vex, obstacles, radius, matGrid):
    Nvex = None
    Nidx = None
    minDist = float("inf")

    for idx, v in enumerate(G.vertices):
        line = Line(v, vex)
        #if isThruObstacle(line, obstacles, radius):
        #    continue
        if isThruObstacleDiscretized(line, matGrid):
            continue
        dist = distance(v, vex)
        if dist < minDist:
            minDist = dist
            Nidx = idx
            Nvex = v

    return Nvex, Nidx

def newVertex(randvex, nearvex, stepSize):
    dirn = np.array(randvex) - np.array(nearvex)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min (stepSize, length)

    newvex = (nearvex[0]+dirn[0], nearvex[1]+dirn[1], nearvex[2] + dirn[2], nearvex[3] + dirn[3])
    return newvex

def isThruObstacleDiscretized(line, unsafe_set):
    seg = Line(line.p, line.p + line.dirn)
    for i in range((int)(line.dist)):
        segStart = seg.p
        segEnd = seg.p + seg.dirn * seg.dist
        if unsafe_set[(int)(seg.p[0])][(int)(seg.p[1])][(int)(seg.p[3])][(int)(seg.p[2])] < 0:
            return True
        seg = Line(seg.p + line.dirn, seg.p + 2 * line.dirn)
    return False

def ObstaclesFromMatLab(matGrid):
    obstacles = []
    for x in range(len(matGrid)):
        for y in range(len(matGrid[0])):
          for z in range(len(matGrid[0][0])):
            for yaw in range(len(matGrid[0][0][0])):
              if matGrid[x][y][z][yaw] <= 0:
                obstacles.append((x, y, yaw, z))
    return obstacles

def smoothPath(path, obstacles):
  """Smooths the given path by removing unnecessary intermediary nodes"""
  if len(path) < 3:
    return path
  leftPointer = 0
  rightPointer = 2
  while rightPointer <= len(path) - 1:
    line = Line(path[leftPointer], path[rightPointer])
    if not isThruObstacleDiscretized(line, obstacles):
      path.remove(path[leftPointer + 1])
    else:
      leftPointer += 1
      rightPointer += 1
  return path

def getPath(start, end, obstaclesFile):
    startpos = start
    endpos = end
    mat = scipy.io.loadmat(obstaclesFile)
    matGrid = mat["data"][:,:,:,:,-1]
    obstacles = ObstaclesFromMatLab(matGrid)
    n_iter = 230
    radius = 5
    stepSize = 4

    G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize, matGrid)

    if G.success:
        path = dijkstra(G)
        path = smoothPath(path, matGrid)
        print(path)
    else:
        print("No path")
    return path

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
    return (newLat, newLong)

def path_to_waypoint(startlat, startlong, startalt, start, path):
    wp = []
    for node in path:
        newLat, newLong = getLatLong(startlat, startlong, node[0] - start[0], node[1] - start[1])

        new_waypoint = {'frame': MavrosDrone.FRAME_REFERENCE.RELATIVE_ALT.value,
                            'command': MavrosDrone.MAV_CMD.NAVIGATE_TO_WAYPOINT.value,
                            'is_current': False, 'autocontinue': True, 'param1': 0,
                            'param2': 0, 'param3': 0, 'param4': node[3], 'x_lat': newLat,
                            'y_long': newLong,
                            'z_alt': node[2]+startalt}
        wp.append(new_waypoint)
    return wp

def getMavrosWaypoints(start, end, obstaclesFile):
    return path_to_waypoint(getPath(start, end, obstaclesFile))
