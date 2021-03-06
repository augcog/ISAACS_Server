from drone import Drone
from sensor import Sensor
import roslibpy
import roslibpy.actionlib
import argparse
import constants
#roslaunch rosbridge_server rosbridge_websocket.launch

#####################
# Global Parameters #
#####################
# Allows option to specify --ip of the server through the command line.
# Ex: python3 operator.py --ip 0.0.0.0
parser = argparse.ArgumentParser(
        description='Starts the operator of the server.')
parser.add_argument('--ip', type=str, default=constants.IP_ADDRESS)
args = parser.parse_args()

# HOST ip parameter
HOST = args.ip

####################
# Global Variables #
####################

drones = dict() # Global map between drone IDs and drone instances
sensors = dict() # Global map between sensor IDs and sensor instances
drone_names = dict() # Global map between drone names and drone IDs
sensor_names = dict() # Global map between sensor names and sensor IDs
all_topics = dict() # Global map of topic names to topic types
# If an id of 0 is passed in, it acts as a wild card.
next_id = 1 # ID to assign next drone or sensor
services = [] # TODO list of all services
latestService = [] # Remembers last service call
#first element is request, second element is response, third element is servicename

###################################
# Set up and boot Roslibpy server #
###################################

ROS_master_connection = roslibpy.Ros(host=HOST, port=9090)
# Use the @custom_service decorator on a handler method to have it
# automatically advertise as a Service.
def custom_service(handler):
    """
    This method is designed to be used as a decorator (@custom_service)
    to advertise the handler method as a service via the ROS_master_connection.
    By default, the service can be found at `/isaacs_server/[handler_name]`
    with a service type of `isaacs_server/[handler_name]`.

    Exceptions for the handler name to service type mapping can be added
    to the exceptions dictionary.

    parameter: handler(request, response) handles an incoming service request.
    returns: handler
    """
    exceptions = {
        'save_drone_topics': 'isaacs_server/type_to_topic',
        'save_sensor_topics': 'isaacs_server/type_to_topic',
        'shutdown_drone': 'isaacs_server/type_to_topic',
        'shutdown_sensor': 'isaacs_server/type_to_topic'
    }
    if handler.__name__ in exceptions:
        serv_type = exceptions[handler.__name__]
    else:
        serv_type = f'isaacs_server/{handler.__name__}'
    service = roslibpy.Service(ROS_master_connection,
            f'/isaacs_server/{handler.__name__}', serv_type)
    print(service.name)
    service.advertise(handler)
    services.append(service)
    return handler

def get_id(client_type):
    '''
    Assigns an new ID to the client type inputted.

    :param client_type: Expects the type of client that an id is assigned to
    (either a Sensor or a Drone)
    This function assigns an id to a Drone or a Sensor.
    The protocol for id assignment is that Drones are odd numbered and
    Sensors are even numbered. (This way it is known what an id corresponds to)
    '''
    global next_id
    if client_type == Drone:
        if next_id % 2 == 1:
            cur_id, next_id = next_id, next_id + 1
        else:
            cur_id, next_id = next_id + 1, next_id + 2
    else:
        if next_id % 2 == 0:
            cur_id, next_id = next_id, next_id + 1
        else:
            cur_id, next_id = next_id + 1, next_id + 2
    return cur_id

def is_drone(client_id):
    '''
    Verifies that the client_id is a drone ID or not.

    :param client_id: id of client to identify
    '''
    if client_id % 2 == 1:
        return True
    return False

def is_sensor(client_id):
    '''
    Verifies that the client_id is a sensor ID or not.

    :param client_id: id of client to identify
    '''
    if client_id % 2 == 0:
        return True
    return False

def checkLatestService(request, serviceName):
    if len(latestService) == 3 and serviceName == latestService[2] and request == latestService[0]:
        print("Repeat service call")
        print(latestService[1])
        print(latestService[1]["success"])
        return True
    return False

def saveLatestService(request, response, serviceName):
    latestService = [request, response, serviceName]

################################
# Interface -> Server Handlers #
################################
@custom_service
def all_drones_available(request, response):

    print("Calling all_drones_available service...")
    if checkLatestService(request, "all_drones_available"):
        response["message"] = latestService[1]["message"]
        response["success"] = latestService[1]["success"]
        response["drones_available"] = latestService[1]["drones_available"]
        return True

    drones_available = []
    for k, v in drones.items():
        avail = {
                "id" : k,
                "name" : v.drone_name,
                "type" : v.drone_type,
                "topics" : v.topics,
                "services" : v.services
        }
        drones_available.append(avail)

    response["success"] = True
    response["message"] = "Successfully sent all available drones to VR"
    response['drones_available'] = drones_available
    print("All_drones_available service finished!", response)

    saveLatestService(request, response, "all_drones_available")
    return True


@custom_service
def upload_mission(request, response):
    '''
    :param request: {id: int, waypoints: list of ints/strings --> pass
    these directly into the drone instance}
    '''
    print("Calling upload_mission service...")
    if checkLatestService(request, "upload_mission"):
        response["message"] = latestService[1]["message"]
        response["success"] = latestService[1]["success"]
        response["id"] = latestService[1]["id"]
        return True

    d = drones.get(request["id"])
    if not d:
        response["success"] = False
        response["message"] = "No drone with that id."
        response["id"] = request["id"]
        saveLatestService(request, response, "upload_mission")
        return True

    callback = d.upload_mission(request["waypoints"])
    response["id"] = d.id
    response["success"] = callback["success"]
    response["message"] = callback["message"]
    print("Upload_mission service finished!")
    saveLatestService(request, response, "upload_mission")
    return True


@custom_service
def set_speed(request, response):
    print("Calling set_speed service...")
    if checkLatestService(request, "set_speed"):
        response["message"] = latestService[1]["message"]
        response["success"] = latestService[1]["success"]
        response["id"] = latestService[1]["id"]
        return True

    d = drones.get(request["id"])
    if not d:
        response["success"] = False
        response["message"] = "No drone with that id."
        response["id"] = request["id"]
        saveLatestService(request, response, "set_speed")
        return True

    print('Setting speed to {}...'.format(request['speed']))
    callback = d.set_speed(request["speed"])
    response["id"] = d.id
    response["success"] = callback["success"]
    response["message"] = callback["message"]
    print("Set_speed service finished!")
    saveLatestService(request, response, "set_speed")
    return True


@custom_service
def control_drone(request, response):
    print("Calling control_drone service...")
    if checkLatestService(request, "control_drone"):
        response["message"] = latestService[1]["message"]
        response["success"] = latestService[1]["success"]
        response["id"] = latestService[1]["id"]
        return True

    control_task = request["control_task"]
    drone = drones.get(request["id"])

    if not drone:
        response["success"] = False
        response["message"] = "Invalid drone id"
        response["id"] = request["id"]
        saveLatestService(request, response, "control_drone")
        return True

    tasks = {
        "start_mission" : drone.start_mission,
        "pause_mission" : drone.pause_mission,
        "resume_mission" : drone.resume_mission,
        "stop_mission" : drone.stop_mission,
        "land_drone" : drone.land_drone,
        "fly_home" : drone.fly_home
    }
    if control_task not in tasks:
        response["success"] = False
        response["message"] = "Invalid control task: " + str(control_task)
        response["id"] = request["id"]
        saveLatestService(request, response, "control_drone")
        return True

    else:
        print(f"Executing {control_task}...")
        callback = tasks.get(control_task)()
        response["id"] = drone.id
        response["success"] = callback["success"]
        response["message"] = callback["message"]
    print("Control_drone service finished!")
    saveLatestService(request, response, "control_drone")
    return True


@custom_service
def query_topics(request, response):
    print("Calling query_topics service...")
    if checkLatestService(request, "query_topics"):
        response["message"] = latestService[1]["message"]
        response["success"] = latestService[1]["success"]
        response["id"] = latestService[1]["id"]
        return True

    client_id = request["id"]
    response["id"] = client_id
    all_topics_response = []
    if client_id == 0:
        for k,v in all_topics.items():
            all_topics_response.append({"name": k, "type": v})
    else:
        if client_id in drones:
            all_topics_response = drones[client_id].topics
            for sensor_id in drones[client_id].sensors:
                all_topics_response += sensors[sensor_id].topics
        elif client_id in sensors:
            all_topics_response = sensors[client_id].topics
        else:
            response["success"] = False
            response["message"] = "No drone or sensor with that id."
            saveLatestService(request, response, "query_topics")
            return True

    response["all_topics"] = all_topics_response
    response["success"] = True
    response["message"] = "Successfully queried topics."
    print(all_topics_response)
    print("Query_topics service finished!")
    print(response)
    saveLatestService(request, response, "query_topics")
    return True

############################
# Drone -> Server Handlers #
############################
@custom_service
def register_drone(request, response):
    '''
    :param request: dict of {drone_name: string, drone_type: string}
    '''
    print("Calling register_drone service...")
    if checkLatestService(request, "register_drone"):
        response["message"] = latestService[1]["message"]
        response["success"] = latestService[1]["success"]
        response["id"] = latestService[1]["id"]
        return True

    drone_name = request["drone_name"]
    drone_type = request["drone_type"]

    if drone_name in drone_names:
        response["success"] = False
        response["message"] = "A drone with this name already exists."
        response["id"] = 0
        print("A drone with the name", drone_name, "already exists")
        saveLatestService(request, response, "register_drone")
        return True

    # Create new drone instance using base class constructor, which should then
    # call child constructor corresponding to the drone_type
    d=Drone.create(drone_name, drone_type, ROS_master_connection)

    if d:
        drone_id = get_id(Drone)
        print(f"Adding drone {id} to global drones map...")
        d.id = drone_id
        drones[drone_id] = d
        drone_names[drone_name] = drone_id
        response["success"] = True
        response["id"] = drone_id
        response["message"] = "Drone registered"
    else:
        response["success"] = False
        response["message"] = "Failed to register drone"
        response["id"] = -1
    print(drones)
    print(drone_names)
    print("Register_drone service finished!")
    saveLatestService(request, response, "register_drone")
    return True


@custom_service
def save_drone_topics(request, response):
    """
    Adds topics to the given drone.

    :param request: message that has a drone id: std_msgs/Int32
        and publishes: issacs_server/topic[]
    This service saves all topics provided into the appropriate drone object.
    """
    print("Calling save_drone_topics...")
    if checkLatestService(request, "save_drone_topics"):
        response["message"] = latestService[1]["message"]
        response["success"] = latestService[1]["success"]
        return True

    publishes = request["publishes"]
    drone_id = request["id"]
    if not drone_id in drones:
        response["success"] = False
        response["message"] = "Drone id does not exist"
        return True
    for topic in publishes:
        all_topics[topic["name"]] = topic["type"]
        drones[drone_id].topics.append(topic)
    response["success"] = True
    response["message"] = "Successfully saved drone topics"
    print(all_topics)
    print("Save_drone_topics service finished!")
    saveLatestService(request, response, "save_drone_topics")
    return True


@custom_service
def shutdown_drone(request, response):
    '''
    Shuts down the drone. Please ensure that the drone is landed.

    :param request: message that has a id: std_msgs/Int32
        and publishes: issacs_server/topic[]
    '''
    print("Calling shutdown_drone service...")
    if checkLatestService(request, "shutdown_drone"):
        response["message"] = latestService[1]["message"]
        response["success"] = latestService[1]["success"]
        return True

    drone_id = request["id"]
    publishes = request["publishes"]
    d = drones.pop(drone_id, None)
    if d:
        drone_names.pop(d.drone_name)
        for topic in publishes:
            all_topics.pop(topic['name'])
        # TODO ensure that drone instance is completely terminated
        d.shutdown()
        response["success"] = True
        response["message"] = "Drone shutdown"
    else:
        response["success"] = False
        response["message"] = "failed to shutdown drone"
        print("Failed to shutdown drone. ID", drone_id, "not found")
        saveLatestService(request, response, "shutdown_drone")
        return True

    print(drone_names)
    print(drones)
    print("Shutdown_drone service finished!")
    saveLatestService(request, response, "shutdown_drone")
    return True


############################
# Sensor -> Server Handlers #
############################
@custom_service
def register_sensor(request, response):
    '''
    :param request: dict of {drone_name: string, drone_type: string}
    Parent drone must be initiated before sensors are registered.
    '''
    print("Calling register_sensor service...")
    if checkLatestService(request, "register_sensor"):
        response["message"] = latestService[1]["message"]
        response["success"] = latestService[1]["success"]
        response["id"] = latestService[1]["id"]
        return True

    sensor_name = request["sensor_name"]
    sensor_type = request["sensor_type"]
    parent_drone_name = request["parent_drone_name"]

    if sensor_name in sensor_names:
        response["success"] = False
        response["message"] = "A sensor with this name already exists."
        response["id"] = 0
        print("A sensor with the name", sensor_name, "already exists")
        saveLatestService(request, response, "register_sensor")
        return True

    s=None
    if parent_drone_name in drone_names:
        parent_drone_id = drone_names[parent_drone_name]
        if parent_drone_id in drones:
            parent_drone_type = drones[parent_drone_id].drone_type
            s = Sensor.create(
                    parent_drone_name, parent_drone_type, parent_drone_id)
    successful = False

    if s:
        sensor_id = get_id(Sensor)
        print(f"Adding sensor {id} to global sensor map.")
        s.id=sensor_id
        sensors[sensor_id] = s
        sensor_names[sensor_name] = sensor_id
        successful = True
        response["success"] = successful
        response["id"] = sensor_id

    if successful:
        response["message"] = "Registering sensor"
    else:
        response["message"] = "Failed to register sensor"
    print(sensor_names)
    print(sensors)
    print("Register_sensor service finished!")
    saveLatestService(request, response, "register_sensor")
    return True


@custom_service
def save_sensor_topics(request, response):
    '''
    :param request: message that has a sensor id: std_msgs/Int32
        and publishes: issacs_server/topic[]
    This adds all of the sensor topics provided into the sensor object.
    This service is called by the sensor client.
    '''
    print("Calling save_sensor_topics service...")
    if checkLatestService(request, "save_sensor_topics"):
        response["message"] = latestService[1]["message"]
        response["success"] = latestService[1]["success"]
        return True

    publishes = request["publishes"]
    sensor_id = request["id"]
    sensor = sensors.get(sensor_id)
    if not sensor:
        response["success"] = False
        response["message"] = "Sensor id does not exist"
        saveLatestService(request, response, "save_sensor_topics")
        return True
    for topic in publishes:
        all_topics[topic["name"]] = topic["type"]
        sensor.topics.append(topic)
    response["success"] = True
    response["message"] = "Successfully saved sensor topics"
    print(all_topics)
    print("Save_sensor_topics service finished!")
    saveLatestService(request, response, "save_sensor_topics")
    return True


@custom_service
def shutdown_sensor(request, response):
    '''
    :param request: message that has a id: std_msgs/Int32
        and publishes: issacs_server/topic[]
    '''
    print("Calling shutdown_sensor service...")
    if checkLatestService(request, "shutdown_sensor"):
        response["message"] = latestService[1]["message"]
        response["success"] = latestService[1]["success"]
        return True

    sensor_id = request["id"]
    publishes = request["publishes"]
    successful = False
    if sensor_id in sensors:
        sensor_names.pop(sensors[sensor_id].sensor_name)
        sensors.pop(sensor_id)
        for topic in publishes:
            all_topics.pop(topic['name'])
        # TODO ensure that sensor instance is completely terminated
        # TODO Remove sensor_subs from global topics dict
        successful = True
        raise NotImplementedError
    response["success"] = successful
    if successful:
        response["message"] = "Sensor successfully shutdown"
    else:
        response["message"] = "Failed to shutdown sensor"
    print(sensor_names)
    print(sensors)
    print("Shutdown_sensor service finished!")

    saveLatestService(request, response, "shutdown_sensor")
    return True


print('Services advertised.')

class ActionServerWorkaround(roslibpy.actionlib.SimpleActionServer):
    def setCustomTopics(self):
        # Sets topic to custom action topics
        self.feedback_publisher = roslibpy.Topic(self.ros, self.server_name + '/Actionfeedback', self.action_name + 'ActionFeedback')
        self.result_publisher = roslibpy.Topic(self.ros, self.server_name + '/Actionresult', self.action_name + 'ActionResult')
        self.goal_listener = roslibpy.Topic(self.ros, self.server_name + '/Actiongoal', self.action_name + 'ActionGoal')
        # Advertise all publishers
        self.feedback_publisher.advertise()
        self.result_publisher.advertise()
        self.goal_listener.subscribe(self._on_goal_message)


server = ActionServerWorkaround(ROS_master_connection, 'isaacs_server/control_drone', 'isaacs_server/control_drone')
server.setCustomTopics()

def execute(goal):
    print("Calling control_drone action...")
    server.send_feedback({"progress": "Calling control_drone action..."})

    control_task = goal["control_task"]
    drone = drones.get(goal["id"])

    tasks = {
        "start_mission" : drone.start_mission,
        "pause_mission" : drone.pause_mission,
        "resume_mission" : drone.resume_mission,
        "stop_mission" : drone.stop_mission,
        "land_drone" : drone.land_drone,
        "fly_home" : drone.fly_home
    }
    print(f"Executing {control_task}...")
    callback = tasks.get(control_task)()
    print("Control_drone service finished!")
    server.send_feedback({"progress": "Control_drone action finished!"})
    server.set_succeeded({"id":drone.id, "success":callback["success"], "message":callback["message"]})




server.start(execute)
print("Start action.")


ROS_master_connection.run_forever()
ROS_master_connection.terminate()
