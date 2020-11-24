import roslibpy
import numpy as np
from drone import Drone
from enum import Enum

class MavrosDrone(Drone):

    drone_type = "DjiMatrice"
    ros_drone_connection = None

    class MAV_CMD(Enum):
        NAVIGATE_TO_WAYPOINT = 16

    class MAV_MODE(Enum):
        MAV_MODE_PREFLIGHT = 0
        MAV_MODE_STABILIZE_DISARMED = 80
        MAV_MODE_STABILIZE_ARMED = 208
        MAV_MODE_MANUAL_DISARMED = 64
        MAV_MODE_MANUAL_ARMED = 192
        MAV_MODE_GUIDED_DISARMED = 88
        MAV_MODE_GUIDED_ARMED = 216
        MAV_MODE_AUTO_DISARMED = 92
        MAV_MODE_AUTO_ARMED = 220
        MAV_MODE_TEST_DISARMED = 66
        MAV_MODE_TEST_ARMED = 194

    def __init__(self, drone_name, drone_type, id=False):
        super().__init__(drone_name, drone_type, id)
        assert(drone_type == self.drone_type)
        self.prev_flight_status = Drone.Flight_Status.NULL

    # TODO Implement
    def upload_mission(self, waypoints):
        self.waypoints = waypoints
        # Converts all the NavSatFix messages to Waypoint so that its MAVROS compatible
        converted_waypoint_objects = []
        for navsatfix in waypoints:
            converted_waypoint_objects.append(roslibpy.Message(convert_navsatfix_mavroswaypoint(navsatfix)))
        try:
            print("Attempting to upload mission...")
            service = roslibpy.Service(self.ROS_master_connection, '/mavros/mission/push', 'mavros_msgs/WaypointPush')
            request = roslibpy.ServiceRequest({'start_index': 0, 'waypoints': converted_waypoint_objects})

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
        waypoint = {'frame': 0, 'command': MavrosDrone.MAV_CMD.NAVIGATE_TO_WAYPOINT, 'is_current': False, 'autocontinue': True, 'param1': 0, 'param2': 0, 'param3': 0}
        waypoint['x_lat'] = navsatfix['latitude']
        waypoint['y_long'] = navsatfix['longitude']
        waypoint['z_alt'] = navsatfix['altitude']
        return waypoint


    def set_speed(self, speed):
        try:
            print("Attempting to set speed...")
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_setSpeed', 'dji_sdk/MissionWpSetSpeed')
            request = roslibpy.ServiceRequest({"speed": speed})

            print('Calling mission_waypoint_setSpeed service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Failed to set new drone speed"}
        # TODO: Upon failure, revert back to original setting
        return result

    def fetch_speed(self):
        try:
            print("Attempting to fetch speed...")
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_getSpeed', 'dji_sdk/MissionWpGetSpeed')
            request = roslibpy.ServiceRequest()

            print('Calling mission_waypoint_setSpeed service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Failed to fetch drone speed"}
        return result

    def start_mission(self):
        try:
            print("Attempting to start drone mission...")
            service = roslibpy.Service(self.ROS_master_connection, '/mavros/set_mode', 'mavros_msgs/SetMode')
            request = roslibpy.ServiceRequest({"base_mode": MavrosDrone.MAV_MODE.MAV_MODE_AUTO_ARMED, "custom_mode": ""})

            print('Calling mission_waypoint_action start service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Mission failed to start"}
        return result
        
    def stop_mission(self):
        try:
            print("Attempting to stop drone mission...")
            service = roslibpy.Service(self.ROS_master_connection, '/mavros/mission/clear', 'mavros_msgs/WaypointClear')
            request = roslibpy.ServiceRequest()

            print('Calling mission_waypoint_action stop service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Mission failed to stop"}
        self.stop_mission_callback(result)
        # TODO: Upon failure, revert back to original setting
        return result

    def stop_mission_callback(self, result):
        # TODO: Add more after figuring out what callback is used to update
        return result["success"]

    # def pause_mission(self):
    #     try:
    #         print("Attempting to pause drone mission...")
    #         service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction')
    #         request = roslibpy.ServiceRequest({"action": Drone.WaypointActions.PAUSE})

    #         print('Calling mission_waypoint_action pause service...')
    #         result = service.call(request)
    #         print('Service response: {}'.format(result))
    #     except:
    #         result = {"success":False, "message":"Mission failed to pause"}
    #     # TODO: Upon failure, revert back to original setting
    #     return result

    # def pause_mission_callback(self, result):
    #     # TODO: Add more after figuring out what callback is used to update
    #     return result["success"]

    # def resume_mission(self):
    #     try:
    #         print("Attempting to resume drone mission...")
    #         service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction')
    #         request = roslibpy.ServiceRequest({"action": Drone.WaypointActions.RESUME})

    #         print('Calling mission_waypoint_action resume service...')
    #         result = service.call(request)
    #         print('Service response: {}'.format(result))
    #     except:
    #         result = {"success":False, "message":"Mission failed to resume"}
    #     # TODO: Upon failure, revert back to original setting
    #     return result
    
    # def resume_mission_callback(self, result):
    #     # TODO: Add more after figuring out what callback is used to update
    #     return result["success"]

    # TODO Implement
    def land_drone(self):
        try:
            print("Attempting to call drone specific service...")
            service = roslibpy.Service(self.ROS_master_connection, '/mavros/cmd/land', 'mavros_msgs/CommandTOL')
            request = roslibpy.ServiceRequest()

            print('Calling land_drone service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Drone landing failed"}
        return result

    def fly_home(self):
        try:
            print("Attempting to call drone specific service...")
            service = roslibpy.Service(self.ROS_master_connection, '/mavros/set_mode', 'mavros_msgs/SetMode')
            request = roslibpy.ServiceRequest({"custom_mode": "RTL"})

            print('Calling fly_home service...')
            #TODO parse service.call(request)
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Drone flying home failed"}
        return result

    #TODO Implement
    def update_mission_helper(self, action):
        if action == Drone.UpdateMissionAction.CONTINUE_MISSION:
            # Call corresponding service and return result
            result = {"success":True, "message":"Vacuously true for testing"}
        elif action == Drone.UpdateMissionAction.UPDATE_CURRENT_MISSION:
            # Call corresponding service and return result
            result = {"success":True, "message":"Vacuously true for testing"}
        elif action == Drone.UpdateMissionAction.END_AND_HOVER:
            # Call corresponding service and return result
            result = {"success":True, "message":"Vacuously true for testing"}
        return result
    
    # TODO: Need to implement update_mission_helper to work
    def update_mission(self):
        if self.flight_status == Drone.Flight_Status.FLYING_HOME:
            result = self.update_mission_helper(Drone.UpdateMissionAction.UPDATE_CURRENT_MISSION)
        elif self.flight_status == Drone.Flight_Status.ON_GROUND_STANDBY:
            if self.prev_flight_status != Drone.Flight_Status.NULL:
                result = self.update_mission_helper(Drone.UpdateMissionAction.CONTINUE_MISSION)
        elif self.flight_status == Drone.Flight_Status.IN_AIR_STANDBY:
            result = self.update_mission_helper(Drone.UpdateMissionAction.CONTINUE_MISSION)
        else:
            result = {"success":False, "message":"Invalid Request: Could not update mission"}
        return result
    
