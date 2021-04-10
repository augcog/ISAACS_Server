'''
Since we don't have the physical drone to advertise its services (land, takeoff, etc)
this script simulates advertising those services. This lets the server actually call them.
Instead of seeing the perform its action, we will get a print message.
'''

import roslibpy
import constants

client = roslibpy.Ros(host=constants.IP_ADDRESS, port=9090)

'''
Simulates advertising the service drone_control from the drone.
TODO: DroneTaskControl should take care of land, takeoff, etc, depending on the input to request["task"] (4 = takeoff, 6 = land, 1 = go home)
We might not need to actually implement each case, depending on how precise we want this "simulator" to be
'''
def drone_control(request, response):
    print("Control task service is being simulated")
    print(f"Performing task: {request['task']}")
    response["result"] = True
    response["cmd_set"] = 0
    response["cmd_id"] = 0
    response["ack_data"] = 0
    return True

def drone_waypoint(request, response):
    print("Drone waypoint service is being simulated")
    print(f"Performing action: {request['action']}")
    response["result"] = True
    response["cmd_set"] = 0
    response["cmd_id"] = 0
    response["ack_datas"] = 0
    return True

service = roslibpy.Service(client, 'isaacs_server/fake_drone_control', 'isaacs_server/FakeDroneControl')
service.advertise(drone_control)

service2 = roslibpy.Service(client, 'isaacs_server/fake_drone_waypoint', 'isaacs_server/FakeDroneWaypoint')
service2.advertise(drone_waypoint)

'''
Simulates advertising the service set_speed from the drone.
'''
def set_speed(request, response):
    print("Set speed service is being simulated")
    print(f"Speed set to {request['speed']}")
    response["result"] = True
    return True

def get_speed(request, response):
    print("Get speed service is being simulated")
    response["speed"] = 420.69
    response["cmd_set"] = 0
    response["cmd_id"] = 0
    response["ack_data"] = 0
    return True

service3 = roslibpy.Service(client, 'isaacs_server/fake_set_speed', 'isaacs_server/FakeSetSpeed')
service3.advertise(set_speed)
service4 = roslibpy.Service(client, 'isaacs_server/fake_get_speed', 'isaacs_server/FakeGetSpeed')
service4.advertise(get_speed)

'''
Simulates advertising the service upload_mission from the drone.
'''
def upload_mission(request, response):
    print("Upload mission service is being simulated")
    print(request["waypoint_task"])
    response["result"] = True
    response["cmd_set"] = 0
    response["cmd_id"] = 0
    response["ack_data"] = 0
    return True

service5 = roslibpy.Service(client, 'isaacs_server/fake_mission_waypoint_upload', 'isaacs_server/FakeMissionWaypointUpload')
service5.advertise(upload_mission)

print("Fake drone services advertised...")

client.run_forever()
client.terminate()
