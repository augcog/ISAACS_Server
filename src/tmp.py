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
#import haversine as hs
import pickle
import sys
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
    #haversineDist = hs.haversine((oldLat, oldLong), (newLat, newLong)) * 1000
    # if abs(haversineDist - trueDist) > 1:
    #     print(trueDist)
    #     print(haversineDist)
    #     print("Error is greater than 1 meter")
    return newLat, newLong


path = [(0.0, 0.0, 0.0, 0.0),
 (27.50361172987735, 5.756929773302058, -0.07769318920107496, 0.0),
 (30, 30, 5, 0)]
wp = []
startpos = (0, 0, 0, 0)
startlat = 37.91522447196717
startlong = -122.33786459393546
endpos = (30,30,5,0)
for node in path:
    newLat, newLong = getLatLong(startlat, startlong, node[0], node[1])

    new_waypoint = {'frame': MavrosDrone.FRAME_REFERENCE.RELATIVE_ALT.value,
                        'command': MavrosDrone.MAV_CMD.NAVIGATE_TO_WAYPOINT.value,
                        'is_current': False, 'autocontinue': True, 'param1': 0,
                        'param2': 0, 'param3': 0, 'param4': node[3], 'x_lat': newLat,
                        'y_long': newLong,
                        'z_alt': node[2]}
    wp.append(new_waypoint)
file = open('wp_small_kcui.txt', 'wb')
pickle.dump(wp, file)
file.close()
