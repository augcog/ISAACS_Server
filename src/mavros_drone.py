import roslibpy
import numpy as np
from drone import Drone
from enum import Enum

class MavrosDrone(Drone):

    drone_type = "MavrosDrone"
    ros_drone_connection = None

    class MAV_CMD(Enum):
        NAVIGATE_TO_WAYPOINT = 16

    def __init__(self, drone_name, drone_type, id=False):
        super().__init__(drone_name, drone_type, id)
        assert(drone_type == self.drone_type)
        self.ros_drone_connection = roslibpy.Ros(host='47.145.22.68', port=9090)
        try:
            self.ros_drone_connection.run(timeout=20)
            position_listener = roslibpy.Topic(self.ros_drone_connection, '/mavros/global_position/global', 'sensor_msgs/NavSatFix')
            position_listener.subscribe(self.received_position_update)
        except:
            print(self.ros_drone_connection.is_connected)
        self.location = None
        self.prev_flight_status = Drone.Flight_Status.NULL

    def received_position_update(self, message):
        self.location = message

    # TODO Implement
    def upload_mission(self, waypoints):
        if not self.location:
            return {"success":False, "message":"Failed to upload waypoints. Drone location is unknown."}
        # TODO NEEDS TO SWITCH OVER TO NAVSATFIX USAGE!
        waypoints = 2*[{'frame': 3, 'command': 22, 'is_current': False, 'autocontinue': True, 'param1': 0, 'param2': 0, 'param3': 0, 'x_lat': self.location['latitude'], 'y_long': self.location['longitude'], 'z_alt': 10}] + waypoints
        self.waypoints = waypoints
        # Converts all the NavSatFix messages to Waypoint so that its MAVROS compatible
        # converted_waypoint_objects = []
        # for navsatfix in waypoints:
        #     converted_waypoint_objects.append(roslibpy.Message(convert_navsatfix_mavroswaypoint(navsatfix)))
        try:
            print("Attempting to upload mission...")
            service = roslibpy.Service(self.ros_drone_connection, '/mavros/mission/push', 'mavros_msgs/WaypointPush')
            request = roslibpy.ServiceRequest({'waypoints': self.waypoints})

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
        waypoint = {'frame': 0, 'command': MavrosDrone.MAV_CMD.NAVIGATE_TO_WAYPOINT.value, 'is_current': False, 'autocontinue': True, 'param1': 0, 'param2': 0, 'param3': 0}
        waypoint['x_lat'] = navsatfix['latitude']
        waypoint['y_long'] = navsatfix['longitude']
        waypoint['z_alt'] = navsatfix['altitude']
        return waypoint


    def set_speed(self, speed):
        try:
            print("Attempting to set speed...")
            service = roslibpy.Service(self.ros_drone_connection, '/mavros/cmd/command', 'mavros_msgs/CommandLong')
            request = roslibpy.ServiceRequest({"command": 178, "param1": 0, "param2": speed, "param3": -1, "param4": 0})

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
        except:
            result = {"success":False, "message":"Mission failed to stop"}
        self.stop_mission_callback(result)
        # TODO: Upon failure, revert back to original setting
        return result

    def stop_mission_callback(self, result):
        # TODO: Add more after figuring out what callback is used to update
        return result["success"]

    def pause_mission(self):
        try:
            print("Attempting to pause drone mission...")
            service = roslibpy.Service(self.ros_drone_connection, '/mavros/cmd/command', 'mavros_msgs/CommandLong')
            request = roslibpy.ServiceRequest({'command': 252, 'param1': 0, 'param2': 2})

            print('Calling pause mission service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Mission failed to pause"}
        # TODO: Upon failure, revert back to original setting
        return result

    def resume_mission(self):
        try:
            print("Attempting to resume drone mission...")
            service = roslibpy.Service(self.ros_drone_connection, '/mavros/set_mode', 'mavros_msgs/SetMode')
            request = roslibpy.ServiceRequest({"custom_mode": "AUTO"})

            print('Calling mission_waypoint_action resume service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Mission failed to resume"}
        # TODO: Upon failure, revert back to original setting
        return result

    def land_drone(self):
        try:
            print("Attempting to call mavros drone specific service...")
            service = roslibpy.Service(self.ros_drone_connection, '/mavros/cmd/land', 'mavros_msgs/CommandTOL')
            request = roslibpy.ServiceRequest()

            print('Calling mavros_land_drone service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Drone landing failed"}
        return result

    def fly_home(self):
        try:
            print("Attempting to call drone specific service...")
            service = roslibpy.Service(self.ros_drone_connection, '/mavros/set_mode', 'mavros_msgs/SetMode')
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
    
