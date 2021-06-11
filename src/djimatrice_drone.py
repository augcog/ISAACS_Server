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

    def __init__(self, drone_name, drone_type, ROS_master_connection, id=None):
        super().__init__(drone_name, drone_type, ROS_master_connection, id)
        assert(drone_type == self.drone_type)
        self.prev_flight_status = Drone.Flight_Status.NULL

    # Takes a list of waypoints for drone to follow.
    # Converts waypoints to DJI format(Waypoint Mission Task)
    # Makes a service call to DJI_SDK mission_waypoint_upload.
    # Returns dictionary describing if service call was successful.
    def upload_mission(self, waypoints):
        self.waypoints = waypoints
        self.mission_msg_list = []
        waypointTask = self.create_waypoint_task(waypoints)
        result = self.upload_waypoint_task(waypointTask)
        return result

    # Helper function for upload_mission()
    # Converts Waypoints to Mission Waypoint Task
    # Mission Waypoint Task is the Proper Format for DJI Service Requests
    # Returns the Mission Waypoint Task
    def create_waypoint_task(self, waypoints):
        command_list = []
        command_params = []
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

    # Helper function for upload_mission().
    # Make a Service Call to DJI_SDK upload_mission
    # Return a dictionary revealing whether the call was successful or not
    def upload_waypoint_task(self, task):
        try:
            print("Attempting to upload waypoint task...")
            # fake_mission_waypoint_upload found in srv folder. Copied directly from DJI SDK for local testing. 
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_upload', 'dji_sdk/MissionWpUpload')
            # service = roslibpy.Service(self.ROS_master_connection, 'isaacs_server/fake_mission_waypoint_upload',
            #         'isaacs_server/FakeWaypointMissionUpload')
            request = roslibpy.ServiceRequest({"waypoint_task": task})

            print('Calling mission_waypoint_upload service...')
            result = service.call(request)
            if result["result"]:
                result = {"success":True, "message":"Upload mission successful"}
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Upload mission failed"}
        return result

    # Takes a float32 number as the speed to set drone too
    # Makes a service call to mission_waypoint_setSpeed in DJI SDK
    # Returns a dictionary describing whether service call was successful
    def set_speed(self, speed):
        try:
            print("Attempting to set speed...")
            # fake_set_speed found in srv folder. Copied directly from DJI SDK for local testing. 
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_setSpeed', 'dji_sdk/MissionWpSetSpeed')
            # service = roslibpy.Service(self.ROS_master_connection, 'isaacs_server/fake_set_speed',
            #         'isaacs_server/FakeSetSpeed')
            request = roslibpy.ServiceRequest({"speed": speed})

            print('Calling mission_waypoint_setSpeed service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
            if result["result"]:
                result = {"success":True, "message":"New drone speed set"}
            else:
                result = {"success":False, "message":"Failed to set new drone speed"}
        except:
            result = {"success":False, "message":"Failed to set new drone speed"}
        return result

    # Gets the Speed of the drone at its current waypoint.
    # Makes a service call to DJI SDK mission_waypoint_getSpeed
    # Returns a dictionary describing if service call was successful.
    # Dictionary has a key "speed" which is a float32 of the current speed. This param is 0 if service call failed.
    def get_speed(self):
        try:
            print("Attempting to fetch speed...")
            # fake_get_speed found in srv folder. Copied directly from DJI SDK for local testing.
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_getSpeed', 'dji_sdk/MissionWpGetSpeed')
            # service = roslibpy.Service(self.ROS_master_connection, 'isaacs_server/fake_get_speed',
            #         'isaacs_server/FakeGetSpeed')
            request = roslibpy.ServiceRequest()

            print('Calling mission_waypoint_setSpeed service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
            if result["speed"] >= 0:
                result = {"success":True, "message":"New drone speed set", "speed": result["speed"]}
            else:
                result = {"success":False, "message":"Failed to set new drone speed", "speed":0}
        except:
            result = {"success":False, "message":"Failed to fetch drone speed", "speed":0}
        return result


    # Starts a Waypoint Mission
    # Makes a service call to mission_waypoint_action in DJI SDK
    # Pass Drone.WaypointActions.START as the action.
    # Returns a dictionary describing whether service call was successful
    def start_mission(self):
        try:
            print("Attempting to start drone mission...")
            # fake_drone_waypoint found in srv folder. Copied directly from DJI SDK for local testing.
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction')
            # service = roslibpy.Service(self.ROS_master_connection, 'isaacs_server/fake_drone_waypoint',
            #         'isaacs_server/FakeDroneWaypoint')
            request = roslibpy.ServiceRequest({"action": Drone.WaypointActions.START})

            print('Calling mission_waypoint_action start service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
            if result["result"]:
                result = {"success":True, "message":"Start mission successful"}
            else:
                result = {"success":False, "message":"Mission failed to start"}
        except Exception as e:
            result = {"success":False, "message":"Mission failed to start"}
            print(e)
        return result

    # Stops a Waypoint Mission
    # Makes a service call to mission_waypoint_action in DJI SDK
    # Pass Drone.Waypoint.Actions.STOP as the action.
    # Returns a dictionary describing whether service call was successful
    
    def stop_mission(self):
        try:
            print("Attempting to stop drone mission...")
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction')
            # fake_drone_waypoint found in srv folder. Copied directly from DJI SDK for local testing.
            # service = roslibpy.Service(self.ROS_master_connection, 'isaacs_server/fake_drone_waypoint',
            #         'isaacs_server/FakeDroneWaypoint')
            request = roslibpy.ServiceRequest({"action": Drone.WaypointActions.STOP})

            print('Calling mission_waypoint_action stop service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
            if result["result"]:
                result = {"success":True, "message":"Stop mission successful"}
            else:
                result = {"success":False, "message":"Mission failed to stop"}
        except:
            result = {"success":False, "message":"Mission failed to stop"}
        return result

    # Pauses a Waypoint Mission
    # Makes a service call to mission_waypoint_action in DJI SDK
    # Pass Drone.Waypoint.Actions.Pause as the action.
    # Returns a dictionary describing whether service call was successful
    def pause_mission(self):
        try:
            print("Attempting to pause drone mission...")
            # fake_drone_waypoint found in srv folder. Copied directly from DJI SDK for local testing.
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction')
            # service = roslibpy.Service(self.ROS_master_connection, 'isaacs_server/fake_drone_waypoint',
            #         'isaacs_server/FakeDroneWaypoint')
            request = roslibpy.ServiceRequest({"action": Drone.WaypointActions.PAUSE})

            print('Calling mission_waypoint_action pause service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
            if result["result"]:
                result = {"success":True, "message":"Pause mission successful"}
            else:
                result = {"success":False, "message":"Mission failed to pause"}
        except:
            result = {"success":False, "message":"Mission failed to pause"}
        return result


    # Resumes a Waypoint Mission
    # Makes a service call to mission_waypoint_action in DJI SDK
    # Pass Drone.Waypoint.Actions.Resume as the action.
    # Returns a dictionary describing whether service call was successful
    def resume_mission(self):
        try:
            print("Attempting to resume drone mission...")
            # fake_drone_waypoint found in srv folder. Copied directly from DJI SDK for local testing.
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction')
            # service = roslibpy.Service(self.ROS_master_connection, 'isaacs_server/fake_drone_waypoint',
            #         'isaacs_server/FakeDroneWaypoint')
            request = roslibpy.ServiceRequest({"action": Drone.WaypointActions.RESUME})

            print('Calling mission_waypoint_action resume service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
            if result["result"]:
                result = {"success":True, "message":"Resume mission successful"}
            else:
                result = {"success":False, "message":"Mission failed to resume"}
        except:
            result = {"success":False, "message":"Mission failed to resume"}
        return result


    # Tells Drone to Land
    # Makes a service call to drone_task_control in DJI SDK
    # Pass Drone.TaskControl.Land as the predefined task.
    # Returns a dictionary describing whether service call was successful
    def land_drone(self):
        try:
            print("Attempting to call drone specific service...")
            # fake_drone_control found in srv folder. Copied directly from DJI SDK for local testing.
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/drone_task_control', 'dji_sdk/DroneTaskControl')
            # service = roslibpy.Service(self.ROS_master_connection, 'isaacs_server/fake_drone_control',
            #         'isaacs_server/FakeDroneControl')
            request = roslibpy.ServiceRequest({"task": Drone.TaskControl.LAND})

            print('Calling land_drone service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
            if result["result"]:
                result = {"success":True, "message":"Land drone successful"}
            else:
                result = {"success":False, "message":"Drone failed to land"}
        except Exception as e:
            result = {"success":False, "message":"Drone landing failed"}
            print(e)
        return result

    # Tells Drone to Fly Home(home is predefined)
    # Makes a service call to drone_task_control in DJI SDK
    # Pass Drone.TaskControl.GO_HOME as the predefined task.
    # Returns a dictionary describing whether service call was successful
    def fly_home(self):
        try:
            print("Attempting to call drone specific service...")
            # fake_drone_control found in srv folder. Copied directly from DJI SDK for local testing.
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/drone_task_control', 'dji_sdk/DroneTaskControl')
            # service = roslibpy.Service(self.ROS_master_connection, 'isaacs_server/fake_drone_control',
            #         'isaacs_server/FakeDroneControl')
            request = roslibpy.ServiceRequest({"task": Drone.TaskControl.GO_HOME})

            print('Calling fly_home service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
            if result["result"]:
                result = {"success":True, "message":"Fly home successful"}
            else:
                result = {"success":False, "message":"Drone failed to fly home"}
        except Exception as e:
            result = {"success":False, "message":"Drone flying home failed"}
            print(e)
        return result

    #TODO
    def shutdown(self):
        result = {"success": True, "message": "Drone shutdown successful"}
        return result
        try:
            print("Attempting to shutdown drone ...")
            # TODO: No shutdown in dji_sdk, running stop mission, land, disable arm control,  for now
            result_stop_mission = self.stop_mission()
            result_land = self.land_drone()
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/drone_arm_control', 'dji_sdk/DroneArmControl')
            request = roslibpy.ServiceRequest({"arm": 0})

            print('Calling disable arm control service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
            if result['success']:
                # Drone arm control successfully disabled
                result = {"success": True, "message": "Drone shutdown successful"}
        except:
            result = {"success":False, "message":"Drone failed to shutdown"}
        return result
