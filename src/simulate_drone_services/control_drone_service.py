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
def handler(request, response):
    print("Control task service is being simulated")
    print(f"Performing task: {request['task']}")
    response["success"] = True
    response["message"] = f"Control task {request['task']} completed"
    return True

service = roslibpy.Service(client, '/fake_drone_control', 'isaacs_server/fake_drone_control')
service.advertise(handler)

print("Control tasks service advertised...")

client.run_forever()
client.terminate()
