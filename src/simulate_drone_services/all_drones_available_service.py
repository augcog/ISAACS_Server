'''
Since we don't have the physical drone to advertise its services (land, takeoff, etc)
this script simulates advertising those services. This lets the server actually call them.
Instead of seeing the perform its action, we will get a print message.
'''

import roslibpy
import json
import ..constants

client = roslibpy.Ros(host=constants.IP_ADDRESS, port=9090)

'''
Simulates advertising the service upload_mission from the drone.
'''
def handler(request, response):
    print("All drones available service is being simulated")
    response["success"] = True
    response["message"] = "Available drones sent"
    ret = dict()
    ret[0] = "random_drone_name"
    response["drones_available"] = [json.dumps(ret)]
    return True

service = roslibpy.Service(client, '/all_drones_available', 'isaacs_server/all_drones_available')
service.advertise(handler)

print("All drones available service advertised...")

client.run_forever()
client.terminate()
