import roslibpy
from abc import ABC, abstractmethod
from enum import Enum

class Sensor(ABC):

    def __init__(self, sensor_name, sensor_type, parent_drone_id, id=None):
        self.sensor_name = sensor_name
        self.sensor_type = sensor_type
        self.parent_drone_id = parent_drone_id
        self.id = id
        self.topics = []
        self.services = []
        self.ROS_master_connection = ROS_master_connection
        self.sensor_namespace = '/sensor_' + str(self.id)

    @staticmethod
    def create(sensor_name, sensor_type, parent_drone_id, id=None):
        from depth_camera_sensor import DepthCamera
        sensors = {
            "Depth Camera": DepthCamera
        }
        if sensor_type not in sensors:
            return False
        else:
            return sensors.get(sensor_type)(sensor_name, sensor_type, parent_drone_id)

    @abstractmethod
    def shutdown(self):
        '''
        Shuts down the sensor and disconnects from ROSBridge.
        Parameters:
            None
        Return:
            dictionary {
                success: boolean
                message: descriptive string
            }
        '''
        pass
