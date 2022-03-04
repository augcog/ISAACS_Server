import roslibpy
import numpy as np
from drone import Drone
from enum import Enum
from rrt_star import getSafeWaypoints

class MavrosDrone(Drone):

    drone_type = "Mavros"

    class MAV_CMD(Enum):
        '''
        For MAV_CMD numbers, see MAVLink Commands (MAV_CMD) section of MAVLink
        protocol: https://mavlink.io/en/messages/common.html#mav_commands
        '''
        NAVIGATE_TO_WAYPOINT = 16
        TAKEOFF = 22
        SET_SPEED = 178

    class FRAME_REFERENCE(Enum):
        '''
        Coordinate frame is global in terms of absolute GPS coords. This GLOBAL
        frame is used by latitude and longitude. However, altitude uses the
        RELATIVE_ALT frame for takeoff purposes. RELATIVE_ALT's value is
        arbitrary, but must be above 1 for simulation purposes.
        '''
        GLOBAL = 0
        RELATIVE_ALT = 3

    def __init__(self, drone_name, drone_type, ROS_master_connection, id=False):
        super().__init__(drone_name, drone_type, ROS_master_connection, id)
        assert(drone_type == self.drone_type)
        self.position= None
        self.prev_flight_status = Drone.Flight_Status.NULL

    def received_position_update(self, message):
        self.position= message

    # Takes a list of waypoints for drone to follow
    # Converts waypoints to MAVROS compattible format
    # Makes a service call to MAVROS mavros_msgs/WaypointPush
    # Returns dictionary describing if service call was successful
    def upload_mission(self, waypoints):
        self.waypoints = waypoints

        # Converts all the NavSatFix messages to Waypoint so that
        # they're MAVROS compatible
        converted_waypoint_objects = []
        for navsatfix in waypoints:
            converted_waypoint_objects.append(
                self.convert_navsatfix_mavroswaypoint(navsatfix))

        # Two takeoff commands prepended to the waypoint list for safety
        # converted_waypoint_objects = 2 * [
        #     {'frame': MavrosDrone.FRAME_REFERENCE.RELATIVE_ALT.value,
        #     'command': MavrosDrone.MAV_CMD.TAKEOFF.value, 'is_current': False,
        #     'autocontinue': True, 'param1': 0, 'param2': 0, 'param3': 0,
        #     'x_lat': self.position['latitude'],
        #     'y_long': self.position['longitude'],
        #     'z_alt': 10}
        #     ] + converted_waypoint_objects
        print(converted_waypoint_objects)
        
        #Comment/Uncomment following line to turn on and off RRT safety
        converted_waypoint_objects = getSafeWaypoints(converted_waypoint_objects)

        try:
            print("Attempting to upload mission...")
            service = roslibpy.Service(self.ROS_master_connection,
                                       self.drone_namespace + '/mavros/mission/push',
                                       'mavros_msgs/WaypointPush')
            request = roslibpy.ServiceRequest(
                {'waypoints': converted_waypoint_objects})

            print('Calling /mavros/mission/push service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
            if result['success']:
                result = {"success": True, "message": "Mission uploaded"}
            else:
                result = {"success": False, "message": "Mission failed to uploaded"}
        except:
            result = {"success": False,
                      "message": "Failed to upload waypoints"}

        return result

    # Helper method for upload_mission()
    # Converts the given waypoints in NavSatFix format and converts to mavros friendly format
    def convert_navsatfix_mavroswaypoint(self, navsatfix):
        '''
        Takes in a NavSatFix message and returns a
        mavros_msgs/Waypoint message as a dictionary.
        '''
        waypoint = {'frame': MavrosDrone.FRAME_REFERENCE.RELATIVE_ALT.value,
                    'command': MavrosDrone.MAV_CMD.NAVIGATE_TO_WAYPOINT.value,
                    'is_current': False, 'autocontinue': True, 'param1': 0,
                    'param2': 0, 'param3': 0, 'x_lat': navsatfix['latitude'],
                    'y_long': navsatfix['longitude'],
                    'z_alt': navsatfix['altitude']}

        return waypoint

    # Takes a float32 numbers as the speed to set drone to
    # Makes a service call to MAVROS mavros_msgs/CommandLong
    # Returns dictionary describing if service call was successful
    def set_speed(self, speed):
        try:
            print("Attempting to set speed...")
            service = roslibpy.Service(self.ROS_master_connection,
                                       self.drone_namespace + '/mavros/cmd/command',
                                       'mavros_msgs/CommandLong')
            request = roslibpy.ServiceRequest(
                {"command": MavrosDrone.MAV_CMD.SET_SPEED.value,
                 "param1": 0, "param2": speed, "param3": -1, "param4": 0})
            print('Calling mission_waypoint_setSpeed service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success": False,
                      "message": "Failed to set new drone speed"}
        return result

    def get_speed(self):
        raise NotImplementedError

    # Starts a Waypoint Mission
    # Makes appropriate MAVROS Service calls that lead to start_mission and takeoff
    # Returns dictionary describing if service call was successful
    def start_mission(self):
        try:
            print("Attempting to set to loiter...")
            guided_service = roslibpy.Service(self.ROS_master_connection,
                                              self.drone_namespace + '/mavros/set_mode',
                                              'mavros_msgs/SetMode')
            guided_request = roslibpy.ServiceRequest({"custom_mode": "LOITER"})
            guided_service.call(guided_request)

            print("Attempting to arm...")
            arm_service = roslibpy.Service(self.ROS_master_connection,
                                           self.drone_namespace + '/mavros/cmd/arming',
                                           'mavros_msgs/CommandBool')
            arm_request = roslibpy.ServiceRequest({'value': True})
            arm_service.call(arm_request)

            print("Attempting to takeoff...")
            takeoff_service = roslibpy.Service(self.ROS_master_connection,
                                               self.drone_namespace + '/mavros/cmd/takeoff',
                                               'mavros_msgs/CommandTOL')
            takeoff_request = roslibpy.ServiceRequest({'altitude': 3})
            takeoff_service.call(takeoff_request)

            mission_start_service = roslibpy.Service(
                self.ROS_master_connection,
                self.drone_namespace + '/mavros/set_mode',
                'mavros_msgs/SetMode')
            mission_start_request = roslibpy.ServiceRequest(
                {"custom_mode": "AUTO"})
            print('Calling mission_waypoint_action start service...')
            result = mission_start_service.call(mission_start_request)
            print('Service response: {}'.format(result))
            if result['mode_sent']:
                self.prev_flight_status = Drone.Flight_Status.FLYING
                result = {"success": True, "message": "Mission starting"}
            else:
                result = {"success": False, "message": "Mission failed to start"}
        except:
            result = {"success": False, "message": "Mission failed to start"}
        return result

    # Stops a Waypoint Mission
    # Makes a service call to MAVROS mavros_msgs/WaypointClear
    # Returns dictionary describing if service call was successful
    def stop_mission(self):
        try:
            print("Attempting to stop drone mission...")
            service = roslibpy.Service(self.ROS_master_connection,
                                       self.drone_namespace + '/mavros/mission/clear',
                                       'mavros_msgs/WaypointClear')
            request = roslibpy.ServiceRequest()

            print('Calling mission_waypoint_action stop service...')
            result = service.call(request)
            print('Service response: {}'.format(result))

            if result['mode_sent']:
                self.prev_flight_status = Drone.Flight_Status.IN_AIR_STANDBY
                result = {"success": True, "message": "Mission stopped"}
            else:
                result = {"success": False, "message": "Mission failed to stop"}
        except:
            result = {"success": False, "message": "Mission failed to stop"}
        return result

    # Pauses a Waypoint Mission
    # Makes a service call to MAVROS mavros_msgs/SetMode and sets it to GUIDED
    # Returns dictionary describing if service call was successful
    def pause_mission(self):
        try:
            print("Attempting to pause drone mission...")
            service = roslibpy.Service(self.ROS_master_connection,
                                       self.drone_namespace + '/mavros/set_mode',
                                       'mavros_msgs/SetMode')
            request = roslibpy.ServiceRequest({"custom_mode": "GUIDED"})

            print('Calling pause mission service...')
            result = service.call(request)
            print('Service response: {}'.format(result))

            if result['mode_sent']:
                self.prev_flight_status = Drone.Flight_Status.PAUSED_IN_AIR
                result = {"success": True, "message": "Mission paused"}
            else:
                result = {"success": False, "message": "Mission failed to pause"}
        except:
            result = {"success": False, "message": "Mission failed to pause"}
        return result

    # Resumes a Waypoint Mission
    # Makes a service call to MAVROS mavros_msgs/SetMode
    # Returns dictionary describing if service call was successful
    def resume_mission(self):
        try:
            print("Attempting to resume drone mission...")
            service = roslibpy.Service(self.ROS_master_connection,
                                       self.drone_namespace + '/mavros/set_mode',
                                       'mavros_msgs/SetMode')
            request = roslibpy.ServiceRequest({"custom_mode": "AUTO"})

            print('Calling mission_waypoint_action resume service...')
            result = service.call(request)
            print('Service response: {}'.format(result))

            if result['mode_sent']:
                self.prev_flight_status = Drone.Flight_Status.FLYING
                result = {"success": True, "message": "Mission resuming"}
            else:
                result = {"success": False, "message": "Mission failed to resume"}
        except:
            result = {"success": False, "message": "Mission failed to resume"}
        return result

    # Tells Drone to Land
    # Makes a service call to MAVROS mavros_msgs/WaypointClear
    # Returns dictionary describing if service call was successful
    def land_drone(self):
        try:
            print("Attempting to call mavros drone specific service...")
            service = roslibpy.Service(self.ROS_master_connection,
                                       self.drone_namespace + '/mavros/cmd/land',
                                       'mavros_msgs/CommandTOL')
            request = roslibpy.ServiceRequest()

            print('Calling mavros_land_drone service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
            if result['mode_sent']:
                self.prev_flight_status = Drone.Flight_Status.LANDING
                result = {"success": True, "message": "Drone lading"}
            else:
                result = {"success": False, "message": "Drone failed to land"}
        except:
            result = {"success": False, "message": "Drone landing failed"}
        return result

    # Tells Drone to fly-home(home is predefined)
    # Makes a service call to MAVROS mavros_msgs/SetMode
    # Returns dictionary describing if service call was successful
    def fly_home(self):
        print(self.drone_namespace + '/mavros/set_mode')
        try:
            print("Attempting to make drone fly_home...")
            service = roslibpy.Service(self.ROS_master_connection,
                                       self.drone_namespace + '/mavros/set_mode',
                                       'mavros_msgs/SetMode')
            request = roslibpy.ServiceRequest({"custom_mode": "RTL"})

            print('Calling fly_home service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
            if result['mode_sent']:
                self.prev_flight_status = Drone.Flight_Status.FLYING_HOME
                result = {"success": True, "message": "Drone flying home"}
            else:
                result = {"success": False, "message": "Drone failed to fly home"}
        except:
            result = {"success": False, "message": "Drone flying home failed"}
        return result

    # Shutsdown Drone
    # Shutsdown the ROS connection and the drone.
    # Returns dictionary describing if service call was successful
    def shutdown(self):
        try:
            print("Attempting to shutdown drone...")
            service = roslibpy.Service(self.ROS_master_connection,
                                       self.drone_namespace + '/shutdown')
            request = roslibpy.ServiceRequest({})

            print('Calling shutdown service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
            if result['success']:
                result = {"success": True, "message": "Drone shutdown successful"}
            else:
                result = {"success": False, "message": "Drone failed to shutdown"}
        except:
            result = {"success": False, "message": "Drone failed to shutdown"}
        return result
