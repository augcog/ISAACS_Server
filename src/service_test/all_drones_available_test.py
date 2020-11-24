import roslibpy

client = roslibpy.Ros(host='136.25.185.6', port=9090)
client.run()

print(client.is_connected)

service = roslibpy.Service(client, 'isaacs_server/all_drones_available', 'isaacs_server/all_drones_available')
request = roslibpy.ServiceRequest({})

print('Calling all_drones_available service...')
result = service.call(request)
print('Service response: {}'.format(result))

client.terminate()
