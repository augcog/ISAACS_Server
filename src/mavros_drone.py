import roslibpy
import numpy as np
from drone import Drone
from enum import Enum

class MavrosDrone(Drone):

    drone_type = "MavrosDrone"
    ros_drone_connection = None

    class MAV_CMD(Enum):
        NAVIGATE_TO_WAYPOINT = 16
        TAKEOFF = 22
        SET_SPEED = 178

    class FRAME_REFERENCE(Enum):
        GLOBAL = 0 # Coordinate frame is global in terms of absolute GPS coords
        RELATIVE_ALT = 3 # Global coordinates for lat and long, but altitude is relative

    def __init__(self, drone_name, drone_type, id=False):
        super().__init__(drone_name, drone_type, id)
        assert(drone_type == self.drone_type)
        self.ros_drone_connection = roslibpy.Ros(host='47.145.22.68', port=9090)
        try:
            self.ros_drone_connection.run(timeout=20)
            position_listener = roslibpy.Topic(self.ros_drone_connection, '/mavros/global_position/global', 'sensor_msgs/NavSatFix')
            position_listener.subscribe(self.received_position_update)
            self.connection_status = True
        except:
            print(self.ros_drone_connection.is_connected)
        self.location = None
        self.prev_flight_status = Drone.Flight_Status.NULL

    def received_position_update(self, message):
        self.location = message

    def upload_mission(self, waypoints):
        if not self.location:
            return {"success":False, "message":"Failed to upload waypoints. Drone location is unknown."}
        self.waypoints = waypoints
        # Converts all the NavSatFix messages to Waypoint so that its MAVROS compatible
        converted_waypoint_objects = []
        for navsatfix in waypoints:
            converted_waypoint_objects.append(self.convert_navsatfix_mavroswaypoint(navsatfix))

        converted_waypoint_objects = 2*[{'frame': MavrosDrone.FRAME_REFERENCE.RELATIVE_ALT.value, 'command': MavrosDrone.MAV_CMD.TAKEOFF.value, 'is_current': False, 'autocontinue': True, 'param1': 0, 'param2': 0, 'param3': 0, 'x_lat': self.location['latitude'], 'y_long': self.location['longitude'], 'z_alt': 10}] + converted_waypoint_objects
        print(converted_waypoint_objects)
        try:
            print("Attempting to upload mission...")
            service = roslibpy.Service(self.ros_drone_connection, '/mavros/mission/push', 'mavros_msgs/WaypointPush')
            request = roslibpy.ServiceRequest({'waypoints': converted_waypoint_objects})

            print('Calling /mavros/mission/push service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Failed to upload waypoints"}
        return result

    def convert_navsatfix_mavroswaypoint(self, navsatfix):
        '''
        Takes in a NavSatFix message and returns a mavros_msgs/Waypoint message.
        This is in the form of a dictionary.
        '''
        waypoint = {'frame': MavrosDrone.FRAME_REFERENCE.RELATIVE_ALT.value, 'command': MavrosDrone.MAV_CMD.NAVIGATE_TO_WAYPOINT.value, 'is_current': False, 'autocontinue': True, 'param1': 0, 'param2': 0, 'param3': 0}
        waypoint['x_lat'] = navsatfix['latitude']
        waypoint['y_long'] = navsatfix['longitude']
        waypoint['z_alt'] = navsatfix['altitude']
        return waypoint


    def set_speed(self, speed):
        try:
            print("Attempting to set speed...")
            service = roslibpy.Service(self.ros_drone_connection, '/mavros/cmd/command', 'mavros_msgs/CommandLong')
            request = roslibpy.ServiceRequest({"command": MavrosDrone.MAV_CMD.SET_SPEED.value, "param1": 0, "param2": speed, "param3": -1, "param4": 0})

            print('Calling mission_waypoint_setSpeed service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Failed to set new drone speed"}
        return result

    def start_mission(self):
        try:
            print("Attempting to set to loiter...")
            guided_service = roslibpy.Service(self.ros_drone_connection, '/mavros/set_mode', 'mavros_msgs/SetMode')
            guided_request = roslibpy.ServiceRequest({"custom_mode": "LOITER"})
            guided_service.call(guided_request)

            print("Attempting to arm...")
            arm_service = roslibpy.Service(self.ros_drone_connection, '/mavros/cmd/arming', 'mavros_msgs/CommandBool')
            arm_request = roslibpy.ServiceRequest({'value': True})
            arm_service.call(arm_request)

            print("Attempting to takeoff...")
            takeoff_service = roslibpy.Service(self.ros_drone_connection, '/mavros/cmd/takeoff', 'mavros_msgs/CommandTOL')
            takeoff_request = roslibpy.ServiceRequest({'altitude': 3})
            takeoff_service.call(takeoff_request)

            mission_start_service = roslibpy.Service(self.ros_drone_connection, '/mavros/set_mode', 'mavros_msgs/SetMode')
            mission_start_request = roslibpy.ServiceRequest({"custom_mode": "AUTO"})
            print('Calling mission_waypoint_action start service...')
            result = mission_start_service.call(mission_start_request)
            print('Service response: {}'.format(result))
            if result['mode_sent']:
                self.prev_flight_status = Drone.Flight_Status.FLYING
        except:
            result = {"success":False, "message":"Mission failed to start"}
        return result

    def stop_mission(self):
        try:
            print("Attempting to stop drone mission...")
            service = roslibpy.Service(self.ros_drone_connection, '/mavros/mission/clear', 'mavros_msgs/WaypointClear')
            request = roslibpy.ServiceRequest()

            print('Calling mission_waypoint_action stop service...')
            result = service.call(request)
            print('Service response: {}'.format(result))

            if result['mode_sent']:
                self.prev_flight_status = Drone.Flight_Status.IN_AIR_STANDBY
        except:
            result = {"success":False, "message":"Mission failed to stop"}
        return result

    def pause_mission(self):
        try:
            print("Attempting to pause drone mission...")
            service = roslibpy.Service(self.ros_drone_connection, '/mavros/set_mode', 'mavros_msgs/SetMode')
            request = roslibpy.ServiceRequest({"custom_mode": "GUIDED"})

            print('Calling pause mission service...')
            result = service.call(request)
            print('Service response: {}'.format(result))

            if result['mode_sent']:
                self.prev_flight_status = Drone.Flight_Status.PAUSED_IN_AIR
        except:
            result = {"success":False, "message":"Mission failed to pause"}
        return result

    def resume_mission(self):
        try:
            print("Attempting to resume drone mission...")
            service = roslibpy.Service(self.ros_drone_connection, '/mavros/set_mode', 'mavros_msgs/SetMode')
            request = roslibpy.ServiceRequest({"custom_mode": "AUTO"})

            print('Calling mission_waypoint_action resume service...')
            result = service.call(request)
            print('Service response: {}'.format(result))

            if result['mode_sent']:
                self.prev_flight_status = Drone.Flight_Status.FLYING
        except:
            result = {"success":False, "message":"Mission failed to resume"}
        return result

    def land_drone(self):
        try:
            print("Attempting to call mavros drone specific service...")
            service = roslibpy.Service(self.ros_drone_connection, '/mavros/cmd/land', 'mavros_msgs/CommandTOL')
            request = roslibpy.ServiceRequest()

            print('Calling mavros_land_drone service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
            if result['success']:
                self.prev_flight_status = Drone.Flight_Status.LANDING
        except:
            result = {"success":False, "message":"Drone landing failed"}
        return result

    def fly_home(self):
        try:
            print("Attempting to make drone fly_home...")
            service = roslibpy.Service(self.ros_drone_connection, '/mavros/set_mode', 'mavros_msgs/SetMode')
            request = roslibpy.ServiceRequest({"custom_mode": "RTL"})

            print('Calling fly_home service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
            if result['mode_sent']:
                self.prev_flight_status = Drone.Flight_Status.FLYING_HOME
        except:
            result = {"success":False, "message":"Drone flying home failed"}
        return result