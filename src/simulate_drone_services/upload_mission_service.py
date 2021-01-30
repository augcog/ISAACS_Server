'''
Since we don't have the physical drone to advertise its services (land, takeoff, etc)
this script simulates advertising those services. This lets the server actually call them.
Instead of seeing the perform its action, we will get a print message.
'''

import roslibpy

client = roslibpy.Ros(host='54.161.15.175', port=9090)

'''
Simulates advertising the service upload_mission from the drone.
'''
def handler(request, response):
    print("Upload mission service is being simulated")
    response["success"] = True
    response["message"] = "Mission uploaded"
    return True

service = roslibpy.Service(client, '/fake_drone_upload_mission', 'isaacs_server/fake_drone_upload_mission')
service.advertise(handler)

print("Upload mission service advertised...")

client.run_forever()
client.terminate()
