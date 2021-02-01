import roslibpy
import time
client = roslibpy.Ros(host='54.161.15.175', port=9090)
client.run()

print(client.is_connected)

service = roslibpy.Service(client, 'isaacs_server/all_drones_available', 'isaacs_server/all_drones_available')
request = roslibpy.ServiceRequest({})

print('Calling all_drones_available service...')
result = service.call(request, callback=lambda r : print("hi", r))
print('Service response: {}'.format(result))

time.sleep(10)

client.terminate()
