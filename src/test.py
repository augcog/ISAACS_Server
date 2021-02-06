import unittest
import roslibpy

'''
Make sure to run operator.py before running make
'''

client = roslibpy.Ros(host='54.161.15.175', port=9090)

class TestVRConnection(unittest.TestCase):

    def test_all_drones_available_dji(self):
        # Register Dji Drone
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "tester1", "drone_type":"DjiMatrice"})
        result = service.call(request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Dji Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # All Drones Available
        service = roslibpy.Service(client, 'isaacs_server/all_drones_available', 'isaacs_server/all_drones_available')
        request = roslibpy.ServiceRequest({})
        result = service.call(request)
        test_drone = {  "id" : uid,
                        "name" : "tester1",
                        "type" : "DjiMatrice",
                        "topics" : {"name": "topicNameDji", "type": "topicType"},
                        "services" : []}
        self.assertTrue(result["success"])
        self.assertIn(test_drone, result['drones_available'])

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # All Drones Available
        service = roslibpy.Service(client, 'isaacs_server/all_drones_available', 'isaacs_server/all_drones_available')
        request = roslibpy.ServiceRequest({})
        result = service.call(request)
        self.assertTrue(result["success"])
        self.assertNotIn(test_drone, result['drones_available'])

    def test_all_drones_available_mavros(self):
        # Register Mavros Drone
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "tester1", "drone_type":"Mavros"})
        result = service.call(request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Mavros Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # All Drones Available
        service = roslibpy.Service(client, 'isaacs_server/all_drones_available', 'isaacs_server/all_drones_available')
        request = roslibpy.ServiceRequest({})
        result = service.call(request)
        test_drone = {  "id" : uid,
                        "name" : "tester1",
                        "type" : "Mavros",
                        "topics" : {"name": "topicNameMavros", "type": "topicType"},
                        "services" : []}
        self.assertTrue(result["success"])
        self.assertIn(test_drone, result['drones_available'])

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # All Drones Available
        service = roslibpy.Service(client, 'isaacs_server/all_drones_available', 'isaacs_server/all_drones_available')
        request = roslibpy.ServiceRequest({})
        result = service.call(request)
        self.assertTrue(result["success"])
        self.assertNotIn(test_drone, result['drones_available'])

    def test_query_topics_dji(self):
        # Register Dji Drone
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "tester1", "drone_type":"DjiMatrice"})
        result = service.call(request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Dji Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Query Topics
        service = roslibpy.Service(client, 'isaacs_server/query_topics', 'isaacs_server/query_topics')
        request = roslibpy.ServiceRequest({"id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])
        self.assertIn({"name": "topicNameDji", "type": "topicType"}, result['all_topics'])

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Query Topics
        service = roslibpy.Service(client, 'isaacs_server/query_topics', 'isaacs_server/query_topics')
        request = roslibpy.ServiceRequest({"id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])
        self.assertNotIn({"name": "topicNameDji", "type": "topicType"}, result['all_topics'])

    def test_query_topics_mavros(self):
        # Register Mavros Drone
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "tester1", "drone_type":"Mavros"})
        result = service.call(request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Mavros Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Query Topics
        service = roslibpy.Service(client, 'isaacs_server/query_topics', 'isaacs_server/query_topics')
        request = roslibpy.ServiceRequest({"id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])
        self.assertIn({"name": "topicNameMavros", "type": "topicType"}, result['all_topics'])

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Query Topics
        service = roslibpy.Service(client, 'isaacs_server/query_topics', 'isaacs_server/query_topics')
        request = roslibpy.ServiceRequest({"id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])
        self.assertNotIn({"name": "topicNameMavros", "type": "topicType"}, result['all_topics'])


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
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
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
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
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
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
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
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
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
