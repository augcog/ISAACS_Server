import roslibpy
import numpy as np
from drone import Drone
from enum import Enum

class DjiMatriceDrone(Drone):

    drone_type = "DjiMatrice"

    class DroneTaskControl(Enum):
        GO_HOME = 1
        TAKEOFF = 4
        LANDING = 6

    def __init__(self, drone_name, drone_type, id=False):
        super().__init__(drone_name, drone_type, id)
        assert(drone_type == self.drone_type)
        self.prev_flight_status = Drone.Flight_Status.NULL

    # TODO Implement
    def upload_mission(self, waypoints):
        self.waypoints = waypoints
        self.mission_msg_list = []
        if self.flight_status == Drone.Flight_Status.ON_GROUND_STANDBY:
            for i in range(len(self.waypoints)):
                way_point_msg = waypoints[i]
                self.mission_msg_list.append(way_point_msg)
            way_point_task = way_point_msg
            self.upload_waypoint_task(way_point_task)
        return False

    def upload_waypoint_task(self, task):
        try:
            print("Attempting to upload waypoint task...")
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_upload', 'dji_sdk/MissionWpUpload')
            request = roslibpy.ServiceRequest({"waypoint_task": task})

            print('Calling mission_waypoint_upload service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Drone landing failed"}
        return result

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
        # TODO: Can start on this once waypoints are implemented

        # if (self.flight_status == Drone.Flight_Status.ON_GROUND_STANDBY):
        #     if (self.prev_flight_status == Drone.Flight_Status.NULL):
        #         self.flight_status = Drone.Flight_Status.FLYING
        #         self.prev_flight_status = Drone.Flight_Status.ON_GROUND_STANDBY
        #         command_list = np.zeros(16)
        #         command_params = np.zeros(16)
        #         for i in range(self.waypoints_count):
        #             waypoint = self.waypoints[i]
        #             waypoint_coord = waypoint.position
        #             # Assumption: position stored as dictionary
        #             waypoint_msg = {}
        #             #TODO: Find out where waypoints are being added
        #     else:
        #         self.update_mission_helper(Drone.UpdateMissionAction.CONTINUE_MISSION)

        try:
            print("Attempting to start drone mission...")
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction')
            request = roslibpy.ServiceRequest({"action": Drone.WaypointActions.START})

            print('Calling mission_waypoint_action start service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Mission failed to start"}
        # self.start_mission_callback(result)
        # TODO: Upon failure, revert back to original setting
        return result

    def start_mission_callback(self, result):
        # TODO: Add more after figuring out what callback is used to update
        return result["success"]
        
    def stop_mission(self):
        try:
            print("Attempting to stop drone mission...")
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction')
            request = roslibpy.ServiceRequest({"action": Drone.WaypointActions.STOP})

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
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction')
            request = roslibpy.ServiceRequest({"action": Drone.WaypointActions.PAUSE})

            print('Calling mission_waypoint_action pause service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Mission failed to pause"}
        # TODO: Upon failure, revert back to original setting
        return result

    def pause_mission_callback(self, result):
        # TODO: Add more after figuring out what callback is used to update
        return result["success"]

    def resume_mission(self):
        try:
            print("Attempting to resume drone mission...")
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction')
            request = roslibpy.ServiceRequest({"action": Drone.WaypointActions.RESUME})

            print('Calling mission_waypoint_action resume service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Mission failed to resume"}
        # TODO: Upon failure, revert back to original setting
        return result
    
    def resume_mission_callback(self, result):
        # TODO: Add more after figuring out what callback is used to update
        return result["success"]

    # TODO Implement
    def land_drone(self):
        try:
            print("Attempting to call drone specific service...")
            #TODO change to actual service call and type
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/drone_task_control', 'dji_sdk/DroneTaskControl')
            # TODO check service type on drone aka check if 6 is correct
            request = roslibpy.ServiceRequest({"task": DroneTaskControl.LAND})

            print('Calling land_drone service...')
            #TODO parse service.call(request)
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Drone landing failed"}
        # self.land_drone_callback(result)
        return result
    
    def land_drone_callback(self, result):
        # TODO: Add more after figuring out what callback is used to update
        return result["success"]

    # TODO Implement
    def fly_home(self):
        try:
            print("Attempting to call drone specific service...")
            #TODO change to actual service call and type
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/drone_task_control', 'dji_sdk/DroneTaskControl')
            # TODO check service type on drone aka check if 1 is correct
            request = roslibpy.ServiceRequest({"task": DroneTaskControl.GO_HOME})

            print('Calling fly_home service...')
            #TODO parse service.call(request)
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Drone flying home failed"}
        return result

    def fly_home_drone_callback(self, result):
        # TODO: Add more after figuring out what callback is used to update
        return result["success"]

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

    def shutdown(self):
        pass
    
