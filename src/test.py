import unittest
import roslibpy

'''
Make sure to run operator.py before running make
'''

client = roslibpy.Ros(host='54.161.15.175', port=9090)

class TestVRConnection(unittest.TestCase):

    def test_upper(self):
        self.assertEqual('foo'.upper(), 'FOO')



class TestDjimatriceCreation(unittest.TestCase):

    def test_register_drone(self):
        # Register Drone
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "tester1", "drone_type":"DjiMatrice"})
        result = service.call(request)
        self.assertTrue(result["success"])

    def test_save_drone_topics(self):
        # Register Drone
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "tester1", "drone_type":"DjiMatrice"})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Save Topics
        publishes = [{"name": "topicName", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": result["id"]})
        result = service.call(request)
        self.assertTrue(result["success"])

    def test_shutdown_drone(self):
        # Register Drone
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "tester1", "drone_type":"DjiMatrice"})
        result = service.call(request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicName", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])


class TestMavrosCreation(unittest.TestCase):

    def test_register_drone(self):
        # Register Drone
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "tester1", "drone_type":"Mavros"})
        result = service.call(request)
        self.assertTrue(result["success"])

    def test_save_drone_topics(self):
        # Register Drone
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "tester1", "drone_type":"Mavros"})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Save Topics
        publishes = [{"name": "topicName", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": result["id"]})
        result = service.call(request)
        self.assertTrue(result["success"])

    def test_shutdown_drone(self):
        # Register Drone
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "tester1", "drone_type":"Mavros"})
        result = service.call(request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicName", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

class TestDjimatriceMethods(unittest.TestCase):

    def test_upper(self):
        self.assertEqual('foo'.upper(), 'FO')

class TestMavrosMethods(unittest.TestCase):

    def test_upper(self):
        self.assertEqual('foo'.upper(), 'FO')

if __name__ == '__main__':
    client.run()
    unittest.main()
    client.terminate()
