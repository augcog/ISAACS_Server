import roslibpy


####################
# Global Variables #
####################

drones = dict() # Global map between drone IDs and drone instances
next_id = 0 # ID to assign next drone


################################
# Interface -> Server Handlers #
################################

def add_drone(request, response):
    '''
    :param request: dict of {ip: string, port: int, drone_type: string}
    '''

    def get_id():
        nonlocal next_id
        cur_id, next_id = next_id, next_id + 1
        return cur_id

    id = get_id()
    print(f"Adding drone {id} to global drones map with following properties:")
    print(f"\tIP: {request['ip']}")
    print(f"\tPort: {request['port']}")
    print(f"\tDroneType: {request['drone_type']}\n")

    # Create new drone instance using base class constructor, which should then
    # call child constructor corresponding to the drone_type (TODO)
    drones[id] = Drone.create(id, ip, port, drone_type)
    response["id"] = id
    response["success"] = drones[id].add_drone()

    return True # TODO check where this return goes to


def upload_mission(request, response):
    '''
    :param request: dict of {id: int, waypoints: list of ints/strings --> pass
    these directly into the drone instance}
    '''

    response["success"] = drones[request["id"]].upload_mission(request["waypoints"])

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


def handler(request, response):
    print('Setting speed to {}'.format(request['data']))
    response['success'] = True
    return True


############################
# Drone -> Server Handlers #
############################





###################################
# Set up and boot Roslibpy server #
###################################

client = roslibpy.Ros(host='136.25.185.6', port=9090)

service = roslibpy.Service(client, '/set_ludicrous_speed', 'std_srvs/SetBool')
service.advertise(handler)

# TODO naming convention for services
add_drone_service = roslibpy.Service(client, '/add_drone', 'isaacs_server/add_drone')
add_drone_service.advertise(add_drone)
print('Services advertised.')

client.run_forever()
client.terminate()
