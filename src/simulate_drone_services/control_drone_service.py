'''
Since we don't have the physical drone to advertise its services (land, takeoff, etc)
this script simulates advertising those services. This lets the server actually call them.
Instead of seeing the perform its action, we will get a print message.
'''

import roslibpy

client = roslibpy.Ros(host='54.161.15.175', port=9090)

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

print("Fake drone services advertised...")

client.run_forever()
client.terminate()
