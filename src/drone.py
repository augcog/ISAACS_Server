import roslibpy

class Drone():
    
    #TODO fix structure
    
    def __init__(self, id, ip, port, drone_type):
        #raise Exception("Drone class not instantiable. Drone type " +
        #                drone_type + " has not been implemented.")
        return
        
    @staticmethod
    def create(id, ip, port, drone_type):
        drones = {
            "DjiMatrice": DjiMatriceDrone
        }
        #TODO Dont create DRONE class with bad drone_type
        return drones.get(drone_type, Drone)(id, ip, port, drone_type)
        
    def add_drone(self):
        return False


class DjiMatriceDrone(Drone):

    drone_type = "DjiMatrice"
    ros_drone_connection = None

    def __init__(self, id, ip, port, drone_type):
        assert(drone_type == self.drone_type)
        self.id = id
        self.ip = ip
        self.port = port

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
