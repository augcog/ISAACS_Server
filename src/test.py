'''
Basic script to test ROS connection.
'''

import roslibpy

client = roslibpy.Ros(host='54.161.15.175', port=9090)

print(client.is_connected)

client.run_forever()
client.terminate()
