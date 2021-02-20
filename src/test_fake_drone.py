import roslibpy

client = roslibpy.Ros(host='54.161.15.175', port=9090)
client.run()

print("Attempting to start drone mission...")
# service = roslibpy.Service(self.ROS_master_connection, 'dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction')
service = roslibpy.Service(client, 'isaacs_server/fake_drone_waypoint', 'isaacs_server/fake_drone_waypoint')
request = roslibpy.ServiceRequest({"action": 0})

print('Calling mission_waypoint_action start service...')
result = service.call(request)
print("Done")

print(result)
