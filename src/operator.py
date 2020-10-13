from drone import Drone
import roslibpy
#roslaunch rosbridge_server rosbridge_websocket.launch

####################
# Global Variables #
####################

drones = dict() # Global map between drone IDs and drone instances
next_id = 0 # ID to assign next drone


################################
# Interface -> Server Handlers #
################################

# Todo Implement
def add_drone(request, response):
    '''
    :param request: dict of {ip: string, port: int, drone_type: string}
    '''

    #TODO error checking fixes ID when bad Drone init
    def get_id():
        global next_id  #TODO FIXME
        cur_id, next_id = next_id, next_id + 1
        return cur_id

    id = get_id()
    ip = request["ip"]
    port = request["port"]
    drone_type = request["drone_type"]
    print(f"Adding drone {id} to global drones map with following properties:")
    print(f"\tIP: {request['ip']}")
    print(f"\tPort: {request['port']}")
    print(f"\tDroneType: {request['drone_type']}\n")

    # Create new drone instance using base class constructor, which should then
    # call child constructor corresponding to the drone_type (TODO)
    drones[id] = Drone.create(id, ip, port, drone_type)
    successful = False
    if drones[id]:
        successful=drones[id].add_drone()
        response["success"] = successful
        response["id"] = id
    #TODO fix message to error
    if successful:
        response["message"] = "Adding drone"
    else:
        response["message"] = "Failed to add drone"

    return True # TODO check where this return goes to

# Todo Implement
def upload_mission(request, response):
    '''
    :param request: dict of {id: int, waypoints: list of ints/strings --> pass
    these directly into the drone instance}
    '''
    if not drones[request["id"]]:
        response["success"] = False
        return False
    response["success"] = drones[request["id"]].upload_mission(request["waypoints"])

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
    drone = drones.get(request["id"])
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





###################################
# Set up and boot Roslibpy server #
###################################

ROS_master_connection = roslibpy.Ros(host='136.25.185.6', port=9090)

service = roslibpy.Service(ROS_master_connection, '/set_ludicrous_speed', 'std_srvs/SetBool')
service.advertise(handler)


# TODO naming convention for services
# Uncomment service advertises as needed
add_drone_service = roslibpy.Service(ROS_master_connection, '/add_drone', 'isaacs_server/add_drone')
add_drone_service.advertise(add_drone)

'''upload_mission_service = roslibpy.Service(ROS_master_connection, '/upload_mission', 'isaacs_server/upload_mission')
upload_mission_service.advertise(upload_mission)

upload_waypoint_task_service = roslibpy.Service(ROS_master_connection, '/upload_waypoint_task', 'isaacs_server/upload_waypoint_task')
upload_waypoint_task_service.advertise(upload_waypoint_task)

set_speed_service = roslibpy.Service(ROS_master_connection, '/set_speed', 'isaacs_server/set_speed')
set_speed_service.advertise(set_speed)'''

control_drone_service = roslibpy.Service(ROS_master_connection, '/control_drone', 'isaacs_server/control_drone')
control_drone_service.advertise(control_drone)

print('Services advertised.')

ROS_master_connection.run_forever()
ROS_master_connection.terminate()
