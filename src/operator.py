from drone import Drone
import roslibpy
#roslaunch rosbridge_server rosbridge_websocket.launch

#####################
# Global Parameters #
#####################
HOST = '136.25.185.6'

####################
# Global Variables #
####################

drones = dict() # Global map between drone IDs and drone instances
sensors = dict() # Global map between sensor IDs and sensor instances
drone_names = dict() # Global map between drone names and drone IDs
sensor_names = dict() # Global map between sensor names and sensor IDs
next_id = 0 # ID to assign next drone or sensor

###################################
# Set up and boot Roslibpy server #
###################################

ROS_master_connection = roslibpy.Ros(host=HOST, port=9090)
services = []
# Use the @custom_service decorator on a handler method to have it automatically advertise as a Service.
def custom_serivce(handler):
    service = roslibpy.Service(ROS_master_connection, f'server/{handler.__name__}', 'isaacs_server/{handler.__name__}')
    service.advertise(handler)
    services.append(service)
    return handler

################################
# Interface -> Server Handlers #
################################

# Todo Implement
def all_drones_available(request, response):
    drones_available = {k: {drone_name: v.name, drone_subs: v.topics} for k, v in drones}
    response["success"]= True
    response['drones_available']= drones_available

    return True


def upload_mission(request, response):
    '''
    :param request: dict of {drone_id: int, waypoints: list of ints/strings --> pass
    these directly into the drone instance}
    '''
    if not drones[request["drone_id"]]:
        response["success"] = False
        return False
    response["success"], response['meta_data'] = drones[request["drone_id"]].upload_mission(request["waypoints"])
    response['drone_id']= request['drone_id']
    return True

# Todo Implement
def upload_waypoint_task(request, response):
    print('Setting speed to {}'.format(request['data']))
    response['success'] = True
    return True

# Todo Implement
def set_speed(request, response):
    print('Setting speed to {}'.format(request['data']))
    response['success'] = True
    return True

# Todo Implement
# includes startmission, pausemission, resume mission, landdrone, flyhome
def control_drone(request, response):
    control_task = request["control_task"]
    drone = drones.get(request["drone_id"])
    if control_task == "start_mission":
        response = drone.start_mission()
    elif control_task == "pause_mission":
        response = drone.pause_mission()
    elif control_task == "resume_mission":
        response = drone.resume_mission()
    elif control_task == "land_drone":
        response = drone.land_drone()
    elif control_task == "fly_home":
        response = drone.fly_home()
    else:
        response["success"] = False
        response["message"] = "Invalid control task"
    return True


''' Funtions to implement
Click button - query drone ID from button
SelectedDrone.waypoints -
Example: MatriceRosDroneConnection
UploadMissionCallback()
StartMission(ID)
StartMissionCallback()
PauseMission(ID)
PauseMissionCallback()
ResumeMission(ID)
ResumeMissionCallback()
LandDrone(ID)
LandDroneCallback()
FlyHome(ID)
FlyHomeCallback()
'''


############################
# Drone -> Server Handlers #
############################

def handler(request, response):
    print('Setting speed to {}'.format(request['data']))
    response['success'] = True
    return True

def register_drone(request, response):
    '''
    :param request: dict of {drone_name: string, drone_type: string}
    '''

    #TODO error checking fixes ID when bad Drone init
    def get_id():
        global next_id  #TODO FIXME
        cur_id, next_id = next_id, next_id + 1
        return cur_id

    drone_name = request["drone_name"]
    drone_type = request["drone_type"]
    print(f"\tDroneType: {request['drone_type']}\n")

    # Create new drone instance using base class constructor, which should then
    # call child constructor corresponding to the drone_type (TODO)
    d=Drone.create(drone_name, drone_type)
    successful=False
    
    if d:
        id = get_id()
        d.id=id
        drones[id] = d
        successful = True
        response["success"] = successful
        response["id"] = id
    print(f"Adding drone {id} to global drones map with following properties:")

    #TODO fix message to error
    if successful:
        response["message"] = "Adding drone"
    else:
        response["message"] = "Failed to add drone"

    return True # TODO check where this return goes to

def shutdown_drone(request, response):
    '''
    :param request: message that has a drone_id: std_msgs/Int32 and drone_subs: issacs_server/topic[]
    '''
    drone_id = request["drone_id"]
    successful = False
    if drone_id in drones:
        drones.pop(drone_id)
        # TODO ensure that drone instance is completely terminated
        # TODO Remove drone_subs from global topics dict
        successful = True
    response["success"] = successful
    if successful:
        response["message"] = "Drone successfully shutdown"
    else:
        response["message"] = "Failed to shutdown drone"



############################
# Sensor -> Server Handlers #
############################

def register_sensor(request, response):
    '''
    :param request: dict of {drone_name: string, drone_type: string}
    '''

    #TODO error checking fixes ID when bad Sensor init
    def get_id():
        global next_id  #TODO FIXME
        cur_id, next_id = next_id, next_id + 1
        return cur_id

    sensor_name = request["sensor_name"]
    sensor_type = request["sensor_type"]
    parent_drone_name = request["parent_drone_message"]
    print(f"\tSensorType: {request['sensor_type']}\n")

    successful=False
    
    # TODO Instantiate Sensor Object
    print(f"Adding sensor {id} to global sensor map with following properties:")

    #TODO fix message to error
    if successful:
        response["message"] = "Adding sensor"
    else:
        response["message"] = "Failed to add sensor"

    return True # TODO check where this return goes to

@custom_serivce
def shutdown_sensor(request, response):
    '''
    :param request: message that has a sensor_id: std_msgs/Int32 and sensor_subs: issacs_server/topic[]
    '''
    sensor_id = request["sensor_id"]
    successful = False
    if sensor_id in sensors:
        sensors.pop(sensor_id)
        # TODO ensure that sensor instance is completely terminated
        # TODO Remove sensor_subs from global topics dict
        successful = True
    response["success"] = successful
    if successful:
        response["message"] = "Sensor successfully shutdown"
    else:
        response["message"] = "Failed to shutdown sensor"


###################################
# Set up and boot Roslibpy server #
###################################

ROS_master_connection = roslibpy.Ros(host='136.25.185.6', port=9090)
services = []
# Use the @custom_service decorator on a handler method to have it automatically advertise as a Service.
def custom_serivce(handler):
    service = roslibpy.Service(ROS_master_connection, f'server/{handler.__name__}', 'isaacs_server/{handler.__name__}')
    service.advertise(handler)
    services.append(service)
    return handler

# TODO naming convention for services
# Uncomment service advertises as needed
register_drone_service = roslibpy.Service(ROS_master_connection, '/register_drone', 'isaacs_server/register_drone')
register_drone_service.advertise(register_drone)

all_drones_available_service = roslibpy.Service(ROS_master_connection, '/all_drones_available', 'isaacs_server/all_drones_available')
all_drones_available_service.advertise(all_drones_available)

upload_mission_service = roslibpy.Service(ROS_master_connection, '/upload_mission', 'isaacs_server/upload_mission')
upload_mission_service.advertise(upload_mission)

'''upload_waypoint_task_service = roslibpy.Service(ROS_master_connection, '/upload_waypoint_task', 'isaacs_server/upload_waypoint_task')
upload_waypoint_task_service.advertise(upload_waypoint_task)

set_speed_service = roslibpy.Service(ROS_master_connection, '/set_speed', 'isaacs_server/set_speed')
set_speed_service.advertise(set_speed)'''

control_drone_service = roslibpy.Service(ROS_master_connection, '/control_drone', 'isaacs_server/control_drone')
control_drone_service.advertise(control_drone)

print('Services advertised.')

ROS_master_connection.run_forever()
ROS_master_connection.terminate()
