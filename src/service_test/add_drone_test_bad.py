import roslibpy

client = roslibpy.Ros(host='136.25.185.6', port=9090)
client.run()

service = roslibpy.Service(client, '/add_drone', 'isaacs_server/add_drone')
request = roslibpy.ServiceRequest({'ip': "some ip", "port": 9090, "drone_type":"spider man"})

print('Calling service...')
result = service.call(request)
print('Service response: {}'.format(result))

client.terminate()
