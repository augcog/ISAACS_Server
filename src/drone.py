import roslibpy
from abc import ABC, abstractmethod
from enum import Enum

class Flight_Status(Enum):
    ON_GROUND_STANDBY = 1
    IN_AIR_STANDBY = 2
    FLYING = 3
    FLYING_HOME = 4
    PAUSED_IN_AIR = 5
    LANDING = 6
    NULL = 7

class Drone(ABC):

    speed = 5
    ROS_master_connection = roslibpy.Ros(host='136.25.185.6', port=9090)

    def __init__(self, id, ip, port, drone_type):
        self.id = id
        self.ip = ip
        self.port = port
        self.drone_type = drone_type
        self.connection_status = False
        self.flight_status = Flight_Status.NULL

        # Don't think we need these
        # self.waypoints = None
        # self.next_waypoint_id = 0
        # self.mission_msg_list = []


    @staticmethod
    def create(id, ip, port, drone_type):
        from djimatrice_drone import DjiMatriceDrone
        drones = {
            "DjiMatrice": DjiMatriceDrone
        }
        if drone_type not in drones:
            return False
        else:
            return drones.get(drone_type)(id, ip, port, drone_type)

    @abstractmethod
    def add_drone(self):
        pass

    @abstractmethod
    def upload_mission(self, waypoints):
        pass

    @abstractmethod
    def upload_waypoint_task(self, task):
        pass

    @abstractmethod
    def set_speed(self, speed):
        pass

    @abstractmethod
    def start_mission(self):
        pass

    @abstractmethod
    def pause_mission(self):
        pass

    @abstractmethod
    def resume_missionk(self):
        pass

    @abstractmethod
    def land_drone(self):
        pass

    @abstractmethod
    def fly_home(self):
        pass
