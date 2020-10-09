import roslibpy
from enum import Enum

class Flight_Status(Enum):
    ON_GROUND_STANDBY = 1
    IN_AIR_STANDBY = 2
    FLYING = 3
    FLYING_HOME = 4
    PAUSED_IN_AIR = 5
    LANDING = 6
    NULL = 7

class Drone():
    
    #TODO fix structure
    
    def __init__(self, id, ip, port, drone_type):
        #raise Exception("Drone class not instantiable. Drone type " +
        #                drone_type + " has not been implemented.")
        self.drones = {
            "DjiMatrice": DjiMatriceDrone
        }
        return
        
    @staticmethod
    def create(id, ip, port, drone_type):
        #TODO Dont create DRONE class with bad drone_type
        if drone_type not in self.drones:
            return False
        else:
            return self.drones.get(drone_type, Drone)(id, ip, port, drone_type)
        
    def add_drone(self):
        return False

    def upload_mission(self, waypoints):
        if drone_type not in self.dones:
            return False
        else:
            return self.drones.get(drone_type, Drone).upload_mission(waypoints)


class DjiMatriceDrone(Drone):

    drone_type = "DjiMatrice"
    ros_drone_connection = None

    def __init__(self, id, ip, port, drone_type):
        assert(drone_type == self.drone_type)
        self.id = id
        self.ip = ip
        self.port = port
        self.waypoints = None
        self.next_waypoint_id = 0
        self.mission_msg_list = []
        self.flight_status = Flight_Status.ON_GROUND_STANDBY

    def add_drone(self):
        try:
            self.ros_drone_connection = roslibpy.Ros(host=self.ip, port=self.port)
            print("connection variable")
            self.ros_drone_connection.run()
            print("connection run")
            return True
        except:
            print("Failure")
            return False

        '''service = roslibpy.Service(client, '/set_ludicrous_speed', 'std_srvs/SetBool')
        service = roslibpy.Service(client, "")

        request =
        result = service.call(request)'''

    def upload_mission(self, waypoints):
        # Find part in source code where you upload an entire mission
        self.waypoints = waypoints
        # TODO: Assumes that upload completely overwrites the old mission
        self.mission_msg_list = []
        if self.flight_status = Flight_Status.ON_GROUND_STANDBY:
            for i in range(len(self.waypoints)):
                # TODO: Get waypoint message from waypoint
                way_point_msg = waypoints[i]
                mission_msg_list.append(way_point_msg)
            # TODO: Create task msg, discuss how to do this
            way_point_task = way_point_msg
            upload_waypoint_task(way_point_task)
        return False

    def upload_waypoint_task(self, task):
        service_name = "/dji_sdk/mission_waypoint_upload"
        # TODO: Clarify what the third argument should be, i.e. what is std_srvs/SetBool
        service = roslibpy.Service(ros_drone_connection, service_name, 'std_srvs/SetBool')
        service.advertise(upload_handler)
        # TODO: Clarify how to get response from handler
        return result

    def start_mission(self, speed=5):
        service_name = "dji_sdk/mission_waypoint_setSpeed"
        service = roslibpy.Service(ros_drone_connection, service_name, 'std_srvs/SetBool')
        service.advertise(speed_handler)
        return result

    def speed_handler(request, response):
        print('Setting speed to {}'.format(request['data']))
        response['success'] = True
        return True

    def upload_handler(request, response):
        print('Return from upload_task')
        response['success'] = True
        return True