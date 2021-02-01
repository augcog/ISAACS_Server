import roslibpy
from abc import ABC, abstractmethod
from enum import Enum

class Drone(ABC):

    class Flight_Status(Enum):
        ON_GROUND_STANDBY = 1
        IN_AIR_STANDBY = 2
        FLYING = 3
        FLYING_HOME = 4
        PAUSED_IN_AIR = 5
        LANDING = 6
        NULL = 7

    class UpdateMissionAction(Enum):
        CONTINUE_MISSION = 0,
        UPDATE_CURRENT_MISSION = 1,
        END_AND_HOVER = 2

    class WaypointActions(Enum):
        START = 0
        STOP = 1
        PAUSE = 2
        RESUME = 3
    
    def __init__(self, drone_name, drone_type, ROS_master_connection, id=None):
        self.id = id
        self.drone_type = drone_type
        self.drone_name = drone_name
        self.connection_status = False
        self.flight_status = Drone.Flight_Status.NULL
        self.topics = []
        self.services = []
        self.sensors = []
        self.mission_msg_list = []
        self.waypoints = []
        self.waypoints_count = 0
        # define position structure as dictionary: {latitude: int, longitude: int}
        self.position = None
        self.ROS_master_connection = ROS_master_connection
        # Speed of drone in flight; default set to 5
        self.speed = 5

    @staticmethod
    def create(drone_name, drone_type, id=None):
        from djimatrice_drone import DjiMatriceDrone
        from mavros_drone import MavrosDrone
        drones = {
            "DjiMatrice": DjiMatriceDrone, 
            "Mavros": MavrosDrone
        }
        if drone_type not in drones:
            return False
        else:
            return drones.get(drone_type)(drone_name, drone_type, id)

    @abstractmethod
    def upload_mission(self, waypoints):
        '''
        Uploads list of waypoints for drone to follow
        Parameters:
            waypoints: List<NavSatFix msgs>
        Return:
            dictionary {
                success: boolean
                message: descriptive string
            }
        '''
        pass

    @abstractmethod
    def set_speed(self, speed):
        '''
        Sets the speed of the drone
        Parameters:
            speed: float32 representing speed to set
        Return:
            dictionary {
                success: boolean
                message: descriptive string
            }
        '''
        pass

    @abstractmethod
    def start_mission(self):
        '''
        Starts waypoint mission
        Parameters:
            None
        Return:
            dictionary {
                success: boolean
                message: descriptive string
            }
        '''
        pass

    @abstractmethod
    def pause_mission(self):
        '''
        Pauses current mission
        Parameters:
            None
        Return:
            dictionary {
                success: boolean
                message: descriptive string
            }
        '''
        pass

    @abstractmethod
    def resume_mission(self):
        '''
        Resumes current mission
        Parameters:
            None
        Return:
            dictionary {
                success: boolean
                message: descriptive string
            }
        '''
        pass

    @abstractmethod
    def land_drone(self):
        '''
        Commands the drone to land
        Parameters:
            None
        Return:
            dictionary {
                success: boolean
                message: descriptive string
            }
        '''
        pass

    @abstractmethod
    def fly_home(self):
        '''
        Commands the drone to fly home
        Parameters:
            None
        Return:
            dictionary {
                success: boolean
                message: descriptive string
            }
        '''
        pass

    @abstractmethod
    def update_mission(self):
        '''
        Updates the current mission
        Parameters:
            None
        Return:
            dictionary {
                success: boolean
                message: descriptive string
            }
        '''
        pass

    @abstractmethod
    def shutdown(self):
        '''
        Shuts down the drone and disconnects from ROSBridge.
        Parameters:
            None
        Return:
            dictionary {
                success: boolean
                message: descriptive string
            }
        '''
        pass 
