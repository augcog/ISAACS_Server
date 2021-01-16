from drone import Drone
from sensor import Sensor
import roslibpy
import argparse
#from topic_types import topic_types
#from drone_msg import drone_msg
#from sensor_msg import sensor_msg
#roslaunch rosbridge_server rosbridge_websocket.launch

#####################
# Global Parameters #
#####################
# Allows option to type --ip and specify ip of the server through the command line. Ex: python3 operator.py --ip 0.0.0.0
parser = argparse.ArgumentParser(description='Starts the operator of the server.')
parser.add_argument('--ip', type=str, default='136.25.185.6')
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
# If an id of 0 is passed in, it refers to all drones, sensors, and global topics
next_id = 1 # ID to assign next drone or sensor

###################################
# Set up and boot Roslibpy server #
###################################

ROS_master_connection = roslibpy.Ros(host=HOST, port=9090)
services = []
# Use the @custom_service decorator on a handler method to have it automatically advertise as a Service.
def custom_service(handler):
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
    service = roslibpy.Service(ROS_master_connection, f'/isaacs_server/{handler.__name__}', serv_type)
    print(service.name)
    service.advertise(handler)
    services.append(service)
    return handler

################################
# Interface -> Server Handlers #
################################
@custom_service
def all_drones_available(request, response):
    drones_available = []
    print("Calling all_drones_available service...")
    for k, v in drones.items():
        avail = dict()
        avail["id"] = k
        avail["name"] = v.drone_name
        avail["type"] = v.drone_type
        avail["topics"] = v.topics
        avail["services"] = v.services
        drones_available.append(avail)

    response["success"] = True
    response["message"] = "Successfully sent all available drones to VR"
    response['drones_available'] = drones_available
    print("All_drones_available service finished!")
    return True


@custom_service
def upload_mission(request, response):
    '''
    :param request: dict of {drone_id: int, waypoints: list of ints/strings --> pass
    these directly into the drone instance}
    '''
    print("Calling upload_mission service...")
    if not drones[request["drone_id"]]:
        response["success"] = False
        response["message"] = "No drone with that id."
        response["drone_id"] = -1
        return False
    response = drones[request["drone_id"]].upload_mission(request["waypoints"])
    print("Upload_mission service finished!")
    return True


@custom_service
def set_speed(request, response):
    print("Calling set_speed service...")
    if not drones[request["drone_id"]]:
        response["success"] = False
        response["message"] = "No drone with that id."
        response["drone_id"] = -1
        return False
    print('Setting speed to {}...'.format(request['data']))
    response = drones[request["drone_id"]].set_speed(request["data"])
    response['success'] = True
    print("Set_speed service finished!")
    return True


# includes startmission, pausemission, resume mission, landdrone, flyhome
@custom_service
def control_drone(request, response):
    print("Calling control_drone service...")
    control_task = request["control_task"]
    drone = drones.get(request["id"])
    if control_task == "start_mission":
        print("Executing start_mission...")
        response = drone.start_mission()
    elif control_task == "pause_mission":
        print("Executing pause_mission...")
        response = drone.pause_mission()
    elif control_task == "resume_mission":
        print("Executing resume_mission...")
        response = drone.resume_mission()
    elif control_task == "land_drone":
        print("Executing land_drone...")
        response = drone.land_drone()
    elif control_task == "fly_home":
        print("Executing fly_home...")
        response = drone.fly_home()
    else:
        response["success"] = False
        response["message"] = "Invalid control task"
        return False
    print("Control_drone service finished!")
    return True


@custom_service
def query_topics(request, response):
    print("Calling query_topics service...")
    id = request["id"]
    response["id"] = id
    all_topics_response = []
    if id == 0:
        for k,v in all_topics.items():
            all_topics_response.append({"name": k, "type": v})
    else:
        if id in drones:
            all_topics_response = drones[id].topics
            for sensor_id in drones[id].sensors:
                all_topics_response += sensors[sensor_id].topics
        elif id in sensors:
            all_topics_response = sensors[id].topics
        else:
            response["success"] = False
            response["message"] = "No drone or sensor with that id."
            return False

    response["all_topics"] = all_topics_response
    response["success"] = True
    response["message"] = "Successfully queried topics."
    print(all_topics_response)
    print("Query_topics service finished!")
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
    def get_id():
        global next_id
        cur_id, next_id = next_id, next_id + 1
        return cur_id
    drone_name = request["drone_name"]
    drone_type = request["drone_type"]
    # Create new drone instance using base class constructor, which should then
    # call child constructor corresponding to the drone_type
    d=Drone.create(drone_name, drone_type)
    successful=False

    if d:
        id = get_id()
        print(f"Adding drone {id} to global drones map...")
        d.id=id
        drones[id] = d
        drone_names[drone_name] = id
        successful = True
        response["success"] = successful
        response["drone_id"] = id
    if successful:
        response["message"] = "Drone registered"
    else:
        response["message"] = "Failed to register drone"
    print(drones)
    print(drone_names)
    print("Register_drone service finished!")
    return True


@custom_service
def save_drone_topics(request, response):
    '''
    :param request: message that has a drone id: std_msgs/Int32 and publishes: issacs_server/topic[]
    This service saves all topics provided into the appropriate drone object.
    '''
    print("Calling save_drone_topics...")
    publishes = request["publishes"]
    id = request["id"]
    if drones[id] == None:
        response["success"] = False
        response["message"] = "Drone id does not exist"
        return False
    for topic in publishes:
        all_topics[topic["name"]] = topic["type"]
        drones[id].topics.append(topic)
    response["success"] = True
    response["message"] = "Successfully saved drone topics"
    print(all_topics)
    print("Save_drone_topics service finished!")
    return True


@custom_service
def shutdown_drone(request, response):
    '''
    :param request: message that has a drone_id: std_msgs/Int32 and drone_subs: issacs_server/topic[]
    '''
    print("Calling shutdown_drone service...")
    id = request["id"]
    publishes = request["publishes"]
    successful = False
    if id in drones:
        drone_names.pop(drones[id].drone_name)
        drones.pop(id)
        for topic in publishes:
            all_topics.pop(topic['name'])
        # TODO ensure that drone instance is completely terminated
        # TODO Remove drone_subs from global topics dict
        successful = True

    response["success"] = successful
    if successful:
        response["message"] = "Drone successfully shutdown"
    else:
        response["message"] = "Failed to shutdown drone"

    print(drone_names)
    print(drones)
    print("Shutdown_drone service finished!")
    return True


############################
# Sensor -> Server Handlers #
############################
@custom_service
def register_sensor(request, response):
    '''
    :param request: dict of {drone_name: string, drone_type: string}
    '''
    print("Calling register_sensor service...")
    def get_id():
        global next_id
        cur_id, next_id = next_id, next_id + 1
        return cur_id
    sensor_name = request["sensor_name"]
    sensor_type = request["sensor_type"]
    parent_drone_name = request["parent_drone_name"]
    parent_drone_id = drone_names[parent_drone_name]
    parent_drone_type = drones[parent_drone_id].drone_type
    s=Sensor.create(parent_drone_name, parent_drone_type, parent_drone_id)
    successful=False

    if s:
        id = get_id()
        print(f"Adding sensor {id} to global sensor map.")
        s.id=id
        sensors[id] = s
        sensor_names[sensor_name] = id
        successful = True
        response["success"] = successful
        response["sensor_id"] = id

    if successful:
        response["message"] = "Registering sensor"
    else:
        response["message"] = "Failed to register sensor"
    print(sensor_names)
    print(sensors)
    print("Register_sensor service finished!")
    return True


@custom_service
def save_sensor_topics(request, response):
    '''
    :param request: message that has a sensor id: std_msgs/Int32 and publishes: issacs_server/topic[]
    This service saves all of the sensor topics provided into the appropriate sensor object.
    '''
    print("Calling save_sensor_topics service...")
    publishes = request["publishes"]
    id = request["id"]
    if sensors[id] == None:
        response["success"] = False
        response["message"] = "Sensor id does not exist"
        return False
    for topic in publishes:
        all_topics[topic["name"]] = topic["type"]
        sensors[id].topics.append(topic)
    response["success"] = True
    response["message"] = "Successfully saved sensor topics"
    print(all_topics)
    print("Save_sensor_topics service finished!")
    return True


@custom_service
def shutdown_sensor(request, response):
    '''
    :param request: message that has a sensor_id: std_msgs/Int32 and sensor_subs: issacs_server/topic[]
    '''
    print("Calling shutdown_sensor service...")
    sensor_id = request["id"]
    publishes = request["publishes"]
    successful = False
    if sensor_id in sensors:
        sensor_names.pop(sensors[id].sensor_name)
        sensors.pop(sensor_id)
        for topic in publishes:
            all_topics.pop(topic['name'])
        # TODO ensure that sensor instance is completely terminated
        # TODO Remove sensor_subs from global topics dict
        successful = True
    response["success"] = successful
    if successful:
        response["message"] = "Sensor successfully shutdown"
    else:
        response["message"] = "Failed to shutdown sensor"
    print(sensor_names)
    print(sensors)
    print("Shutdown_sensor service finished!")
    return True


print('Services advertised.')

ROS_master_connection.run_forever()
ROS_master_connection.terminate()