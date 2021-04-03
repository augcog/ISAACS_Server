import roslibpy

client = roslibpy.Ros(host='54.161.15.175', port=9090)
client.run()

print(client.is_connected)

land_service = roslibpy.Service(client, 'isaacs_server/control_drone', 'isaacs_server/ControlDrone')
land_request = roslibpy.ServiceRequest({"id": 1, "control_task":"land_drone"})

print('Calling land_drone service...')
result = land_service.call(land_request)
print('Service response: {}'.format(result))

client.terminate()
