import roslibpy
from drone import Drone


class DjiMatriceDrone(Drone):

    drone_type = "DjiMatrice"
    ros_drone_connection = None

    def __init__(self, id, ip, port, drone_type):
        super().__init__(id, ip, port, drone_type)
        assert(drone_type == self.drone_type)

    #TODO going to change once we make drones connect to server instead of server to drones
    def add_drone(self):
        try:
            client = roslibpy.Ros(host='136.25.185.6', port=9090)
            client.run()
            print(client.is_connected)
            return True
        except Exception as e:
            print("Failure")
            print(e)
            return False

    # TODO Implement
    def upload_mission(self, waypoints):
        # Find part in source code where you upload an entire mission
        self.waypoints = waypoints
        # TODO: Assumes that upload completely overwrites the old mission
        self.mission_msg_list = []
        if self.flight_status == Flight_Status.ON_GROUND_STANDBY:
            for i in range(len(self.waypoints)):
                # TODO: Get waypoint message from waypoint
                way_point_msg = waypoints[i]
                mission_msg_list.append(way_point_msg)
            # TODO: Create task msg, discuss how to do this
            way_point_task = way_point_msg
            upload_waypoint_task(way_point_task)
        return False

    # TODO Implement
    def upload_waypoint_task(self, task):
        return

    # TODO Implement
    def start_mission(self):
        return

    # TODO Implement
    def set_speed(self, speed):
        return

    # TODO Implement
    def start_mission(self):
        return

    # TODO Implement
    def pause_mission(self):
        return

    # TODO Implement
    def resume_missionk(self):
        return

    # TODO Implement
    def land_drone(self):
        # change service name on drone
        try:
            print("Attempting to call drone specific service...")
            service = roslibpy.Service(self.ROS_master_connection, 'land_drone_' + str(self.id), 'dji_sdk/DroneTaskControl')
            # TODO check service type on drone aka check if 6 is correct
            request = roslibpy.ServiceRequest({"task": 6})

            print('Calling land_drone service...')
            #TODO parse service.call(request)
            result = service.call(request)
            print('Service response: {}'.format(result))
        except:
            result = {"success":False, "message":"Bricked"}
        return result

    # TODO Implement
    def fly_home(self):
        return
