import roslibpy

client = roslibpy.Ros(host='47.145.22.68', port=9090)

print(client.is_connected)

client.run_forever()
client.terminate()
