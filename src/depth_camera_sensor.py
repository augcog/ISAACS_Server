import roslibpy
from sensor import Sensor

class DepthCamera(Sensor):

    def __init__(self, sensor_name, sensor_type, ROS_master_connection, parent_drone_id, id=None):
        super().__init__(sensor_name, sensor_type, ROS_master_connection, parent_drone_id, id)
        assert(sensor_type == self.sensor_type)

    def shutdown(self):
        return
        raise NotImplementedError
