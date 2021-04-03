'''
Since we don't have the physical drone to advertise its services (land, takeoff, etc)
this script simulates advertising those services. This lets the server actually call them.
Instead of seeing the perform its action, we will get a print message.
'''

import roslibpy

client = roslibpy.Ros(host='54.161.15.175', port=9090)

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

service = roslibpy.Service(client, 'isaacs_server/fake_set_speed', 'isaacs_server/fake_set_speed')
service.advertise(set_speed)
service2 = roslibpy.Service(client, 'isaacs_server/fake_get_speed', 'isaacs_server/fake_get_speed')
service2.advertise(get_speed)

print("Set speed service advertised...")

client.run_forever()
client.terminate()
