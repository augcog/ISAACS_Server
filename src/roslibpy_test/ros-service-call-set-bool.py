'''
This is the example calling services script.
Found at: https://roslibpy.readthedocs.io/en/latest/examples.html
'''

import roslibpy

client = roslibpy.Ros(host='54.161.15.175', port=9090)
client.run()

service = roslibpy.Service(client, '/set_ludicrous_speed', 'std_srvs/SetBool')
request = roslibpy.ServiceRequest({'data': True})

print('Calling service...')
result = service.call(request)
print('Service response: {}'.format(result))

client.terminate()
