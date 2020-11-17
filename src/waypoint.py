import roslibpy
import numpy as np
from drone import Drone
from abc import ABC, abstractmethod
from enum import Enum

class Waypoint(ABC):

    class WaypointStatus(Enum):
        STATIC = 0 # Indicates if this waypoint is just static, placed and has not been passed.
        PASSED = 1 # Indicates whether this waypoint has been passed by the drone
        GRABBED = 2 # Indicates whether this waypoint is currently grabbed
        LOCKED = 3 # Indicated whether this waypoint is locked (cannot be edited by the user)
        UPLOADED = 4 # Indicated whether this waypoint has been uploaded to the drone
    
    # Called within drone class/subclass, must pass in self as drone
    def __init__(self, drone, position):
        self.position = position
        self.prev_waypoint = None
        self.next_waypoint = None
        self.drone = drone
        self.waypoint_status = self.WaypointStatus.STATIC
        self.prev_waypoint_status = self.WaypointStatus.STATIC
    
    def waypoint_passed(self):
        self.prev_waypoint_status = self.waypoint_status
        self.waypoint_status = self.WaypointStatus.PASSED

    def lock_waypoint(self):
        if (self.waypoint_status == self.WaypointStatus.PASSED):
            print("Invalid Commnad")
        self.prev_waypoint_status = self.waypoint_status
        self.waypoint_status = self.WaypointStatus.LOCKED

    def unlock_waypoint(self):
        self.prev_waypoint_status = self.waypoint_status
        self.waypoint_status = self.WaypointStatus.STATIC
    
    def waypoint_uploaded(self):
        self.prev_waypoint_status = self.waypoint_status
        self.waypoint_status = self.WaypointStatus.UPLOADED
    
    # TODO: Consider use cases
    # May want to check validity before calling waypoint_passed, bypassing the 
    # need for set_passed_state()
    def set_passed_state(self):
        # TODO: Add check to make sure drone is actually near waypoint
        # before passing
        if (self.waypoint_status != self.WaypointStatus.PASSED"""add check here"""):
            self.prev_waypoint_status = self.waypoint_status
            self.waypoint_status = self.WaypointStatus.PASSED
    

