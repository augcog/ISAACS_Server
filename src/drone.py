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

    speed = 5
    ROS_master_connection = roslibpy.Ros(host='136.25.185.6', port=9090)

    def __init__(self, drone_name, drone_type, id=None):
        self.id = id
        self.drone_type = drone_type
        self.drone_name= drone_name
        self.connection_status = False
        self.flight_status = Drone.Flight_Status.NULL
        self.topics = []
        self.services = []
        self.sensors = []
        self.mission_msg_list = []
        self.waypoints = []
        self.waypoints_count = 0
        #TODO: define position structure i.e. (lat, long) or (x, y);
        # tuple vs dictionary
        self.position = None

    @staticmethod
    def create(drone_name, drone_type, id=None):
        from djimatrice_drone import DjiMatriceDrone
        from mavros_drone import MavrosDrone
        drones = {
            "DjiMatrice": DjiMatriceDrone, 
            "MavrosDrone": MavrosDrone
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
            speed: int representing speed to set
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
