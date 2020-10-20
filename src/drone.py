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

    def __init__(self, drone_name, drone_type, id=None):
        self.id = id
        self.drone_type = drone_type
        self.drone_name= drone_name
        self.connection_status = False
        self.flight_status = Flight_Status.NULL
        self.topics = {}

    @staticmethod
    def create(drone_name, drone_type, id=None):
        from djimatrice_drone import DjiMatriceDrone
        drones = {
            "DjiMatrice": DjiMatriceDrone
        }
        if drone_type not in drones:
            return False
        else:
            return drones.get(drone_type)(drone_name, drone_type, id)

    @abstractmethod
    def upload_mission(self, waypoints):
        '''
        waypoints: List<NavSatFix msgs>
        return success: boolean and meta_data: JSON(raw drone API callback data)
        '''
        return True, {}

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
    def resume_mission(self):
        pass

    @abstractmethod
    def land_drone(self):
        pass

    @abstractmethod
    def fly_home(self):
        pass
