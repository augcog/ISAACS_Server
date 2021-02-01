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

    @staticmethod
    def create(sensor_name, sensor_type, parent_drone_id, id=None):
        sensors = {

        }
        if sensor_type not in sensors:
            return False
        else:
            return sensors.get(sensor_type)(sensor_name, sensor_type, parent_drone_id)
