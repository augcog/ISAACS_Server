import roslibpy
from drone import Drone

class WaypointActions(Enum):
    START = 0
    STOP = 1
    PAUSE = 2
    RESUME = 3

class DjiMatriceDrone(Drone):

    drone_type = "DjiMatrice"
    ros_drone_connection = None

    def __init__(self, drone_name, drone_type, id=False):
        super().__init__(drone_name, drone_type, id)
        assert(drone_type == self.drone_type)

    # TODO Implement
    def upload_mission(self, waypoints):
        # Find part in source code where you upload an entire mission
        self.waypoints = waypoints
        # TODO: Assumes that upload completely overwrites the old mission
        self.mission_msg_list = []
        if self.flight_status == Drone.Flight_Status.ON_GROUND_STANDBY:
            for i in range(len(self.waypoints)):
                # TODO: Get waypoint message from waypoint
                way_point_msg = waypoints[i]
                mission_msg_list.append(way_point_msg)
            # TODO: Create task msg, discuss how to do this
            way_point_task = way_point_msg
            upload_waypoint_task(way_point_task)

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
            result = {"success":False, "message":"Drone landing failed"}
        return result

    def start_mission(self):
        try:
            print("Attempting to start drone mission...")
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction')
            request = roslibpy.ServiceRequest({"action": WaypointActions.START})

            print('Calling mission_waypoint_action start service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Drone landing failed"}
        return result

    def pause_mission(self):
        try:
            print("Attempting to pause drone mission...")
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction')
            request = roslibpy.ServiceRequest({"action": WaypointActions.PAUSE})

            print('Calling mission_waypoint_action pause service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Drone landing failed"}
        return result

    def resume_mission(self):
        try:
            print("Attempting to resume drone mission...")
            service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction')
            request = roslibpy.ServiceRequest({"action": WaypointActions.RESUME})

            print('Calling mission_waypoint_action resume service...')
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Drone landing failed"}
        return result

    # TODO Implement
    def land_drone(self):
        try:
            print("Attempting to call drone specific service...")
            #TODO change to actual service call and type
            service = roslibpy.Service(self.ROS_master_connection, '/fake_drone_control', 'isaacs_server/fake_drone_control')
            # TODO check service type on drone aka check if 6 is correct
            request = roslibpy.ServiceRequest({"task": 6})

            print('Calling land_drone service...')
            #TODO parse service.call(request)
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Drone landing failed"}
        return result

    # TODO Implement
    def fly_home(self):
        try:
            print("Attempting to call drone specific service...")
            #TODO change to actual service call and type
            service = roslibpy.Service(self.ROS_master_connection, '/fake_drone_control', 'isaacs_server/fake_drone_control')
            # TODO check service type on drone aka check if 6 is correct
            request = roslibpy.ServiceRequest({"task": 1})

            print('Calling fly_home service...')
            #TODO parse service.call(request)
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Drone flying home failed"}
        return result
