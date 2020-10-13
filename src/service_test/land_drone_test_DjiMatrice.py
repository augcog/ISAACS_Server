import roslibpy

client = roslibpy.Ros(host='136.25.185.6', port=9090)
client.run()

print(client.is_connected)

service = roslibpy.Service(client, '/add_drone', 'isaacs_server/add_drone')
request = roslibpy.ServiceRequest({'ip': "136.25.185.6", "port": 9090, "drone_type":"DjiMatrice"})

print('Calling add_drone service...')
result = service.call(request)
print('Service response: {}'.format(result))

land_service = roslibpy.Service(client, '/control_drone', 'isaacs_server/control_drone')
land_request = roslibpy.ServiceRequest({"id": 0, "control_task":"land_drone"})

print('Calling land_drone service...')
result = land_service.call(land_request)
print('Service response: {}'.format(result))

client.terminate()
