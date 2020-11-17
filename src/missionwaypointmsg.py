import roslibpy
import numpy as np
from drone import Drone
from abc import ABC, abstractmethod
from enum import Enum

class MissionWaypointMsg(ABC):

    class TurnMode(Enum):
        CLOCKWISE = 0
        COUNTERCLOCKWISE = 1

    def __init__(self, msg=None, latitude, longitude, altitude, damping_distance, 
                yaw, gimbal_pitch, turn_mode, has_action, action_time_limit, waypoint_action):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.damping_distance = damping_distance
        self.yaw = yaw
        self.gimbal_pitch = gimbal_pitch
        self.turn_mode = turn_mode
        self.has_action = has_action
        self.action_time_limit = action_time_limit
        self.waypoint_action = waypoint_action

    
        