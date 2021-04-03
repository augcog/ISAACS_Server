import roslibpy
import numpy as np
from drone import Drone
from enum import IntEnum

class DjiMatriceDrone(Drone):
    '''
    DJI Matrice API translator layer.
    Please see https://github.com/dji-sdk/Onboard-SDK-ROS for more information about the DJI SDK.
    To use this, please install [TODO] on the DJI drone's onboard computer.
    '''

    drone_type = "DjiMatrice"

    '''
    This enum is used to reference control task numbers as hardcoded in the DJI SDK.
    '''
    class DroneTaskControl(IntEnum):
        GO_HOME = 1
        TAKEOFF = 4
        LAND = 6

    def __init__(self, drone_name, drone_type, ROS_master_connection, id=False):
        super().__init__(drone_name, drone_type, ROS_master_connection, id)
        assert(drone_type == self.drone_type)
        self.prev_flight_status = Drone.Flight_Status.NULL

    def upload_mission(self, waypoints):
        self.waypoints = waypoints
        self.mission_msg_list = []
        waypointTask = self.create_waypoint_task(waypoints)
        result = self.upload_waypoint_task(waypointTask)
        return result

    # Helper function for upload_mission()
    def create_waypoint_task(self, waypoints):
        command_list = [];
        command_params = [];
        for i in range(16):
            command_list.append(0)
            command_params.append(0)

        # Mission Waypoint Action
        missionWaypointActionMsg = {"action_repeat":0, "command_list":command_list, "command_parameter":command_params}
        missionWaypoints = []

        #Mission Waypoint
        for wp in waypoints:
            missionWp = dict()
            missionWp["latitude"] = wp["latitude"]
            missionWp["longitude"] = wp["longitude"]
            missionWp["altitude"] = wp["altitude"]
            missionWp["damping_distance"] = 3
            missionWp["target_yaw"] = 0
            missionWp["target_gimbal_pitch"] = 0
            missionWp["turn_mode"] = 0
            missionWp["has_action"] = 0
            missionWp["action_time_limit"] = 30
            missionWp["waypoint_action"] = missionWaypointActionMsg
            missionWaypoints.append(missionWp)

        #Mission Waypoint Task
        missionWaypointTask = dict()
        missionWaypointTask["velocity_range"] = 15
        missionWaypointTask["idle_velocity"] = 15
        missionWaypointTask["action_on_finish"] = 0
        missionWaypointTask["mission_exec_times"] = 1
        missionWaypointTask["yaw_mode"] = 0
        missionWaypointTask["trace_mode"] = 0
        missionWaypointTask["action_on_rc_lost"] = 0
        missionWaypointTask["gimbal_pitch_mode"] = 0
        missionWaypointTask["mission_waypoint"] = missionWaypoints

        return missionWaypointTask

    # Helper function for upload_mission()
    def upload_waypoint_task(self, task):
        try:
            print("Attempting to upload waypoint task...")
            # service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_upload', 'dji_sdk/MissionWpUpload')
            service = roslibpy.Service(self.ROS_master_connection, 'isaacs_server/fake_mission_waypoint_upload', 'isaacs_server/fake_mission_waypoint_upload')
            request = roslibpy.ServiceRequest({"waypoint_task": task})

            print('Calling mission_waypoint_upload service...')
            result = service.call(request)
            if result["result"]:
                result = {"success":True, "message":"Upload mission successful"}
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Upload mission failed"}
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
        try:
            print("Attempting to start drone mission...")
            # service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction')
            service = roslibpy.Service(self.ROS_master_connection, 'isaacs_server/fake_drone_waypoint', 'isaacs_server/fake_drone_waypoint')
            request = roslibpy.ServiceRequest({"action": Drone.WaypointActions.START})

            print('Calling mission_waypoint_action start service...')
            #TODO parse service.call(request)
            result = service.call(request)
            print('Service response: {}'.format(result))
        except Exception as e:
            result = {"success":False, "message":"Mission failed to start"}
            print(e)
        return result

    def stop_mission(self):
        try:
            print("Attempting to stop drone mission...")
            # service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction')
            service = roslibpy.Service(self.ROS_master_connection, 'isaacs_server/fake_drone_waypoint', 'isaacs_server/fake_drone_waypoint')
            request = roslibpy.ServiceRequest({"action": Drone.WaypointActions.STOP})

            print('Calling mission_waypoint_action stop service...')
            #TODO parse service.call(request)
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Mission failed to stop"}
        self.stop_mission_callback(result)
        return result

    def pause_mission(self):
        try:
            print("Attempting to pause drone mission...")
            #service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction')
            service = roslibpy.Service(self.ROS_master_connection, 'isaacs_server/fake_drone_waypoint', 'isaacs_server/fake_drone_waypoint')
            request = roslibpy.ServiceRequest({"action": Drone.WaypointActions.PAUSE})

            print('Calling mission_waypoint_action pause service...')
            #TODO parse service.call(request)
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Mission failed to pause"}
        return result

    def resume_mission(self):
        try:
            print("Attempting to resume drone mission...")
            #service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction')
            service = roslibpy.Service(self.ROS_master_connection, 'isaacs_server/fake_drone_waypoint', 'isaacs_server/fake_drone_waypoint')
            request = roslibpy.ServiceRequest({"action": Drone.WaypointActions.RESUME})

            print('Calling mission_waypoint_action resume service...')
            #TODO parse service.call(request)
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Mission failed to resume"}
        return result

    def land_drone(self):
        try:
            print("Attempting to call drone specific service...")
            #TODO change to actual service call and type
            # service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/drone_task_control', 'dji_sdk/DroneTaskControl')
            service = roslibpy.Service(self.ROS_master_connection, 'isaacs_server/fake_drone_control', 'isaacs_server/fake_drone_control')
            request = roslibpy.ServiceRequest({"task": Drone.TaskControl.LAND})

            print('Calling land_drone service...')
            #TODO parse service.call(request)
            result = service.call(request)
            print('Service response: {}'.format(result))
        except Exception as e:
            result = {"success":False, "message":"Drone landing failed"}
            print(e)
        return result

    def fly_home(self):
        try:
            print("Attempting to call drone specific service...")
            #TODO change to actual service call and type
            #service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/drone_task_control', 'dji_sdk/DroneTaskControl')
            service = roslibpy.Service(self.ROS_master_connection, 'isaacs_server/fake_drone_control', 'isaacs_server/fake_drone_control')
            request = roslibpy.ServiceRequest({"task": Drone.TaskControl.GO_HOME})

            print('Calling fly_home service...')
            #TODO parse service.call(request)
            result = service.call(request)
            print('Service response: {}'.format(result))
        except Exception as e:
            result = {"success":False, "message":"Drone flying home failed"}
            print(e)
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

    #TODO
    def shutdown(self):
        return {"success": True, "message": "Drone shutdown"}
