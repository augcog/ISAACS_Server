import roslibpy
import json

# Since we don't have the physical drone to advertise its services (land, takeoff, etc)
# this script simulates advertising those services. This lets the server actually call them.
# Instead of seeing the drone land, at least we can see a print message saying the drone would be landing if we had one


client = roslibpy.Ros(host='136.25.185.6', port=9090)

#TODO: DroneTaskControl should take care of land, takeoff, etc, depending on the input to request["task"] (4 = takeoff, 6 = land, 1 = go home)
#We might not need to actually implement each case, depending on how precise we want this "simulator" to be
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
