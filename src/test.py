import unittest
import roslibpy
import timeout_decorator

'''
Make sure to restart operator.py before running make
Make sure to make each drone_name unique
'''

TIMEOUT = 30

client = roslibpy.Ros(host='54.161.15.175', port=9090)

class TestVRConnection(unittest.TestCase):
    
    @timeout_decorator.timeout(TIMEOUT)
    def test_all_drones_available_dji(self):
        # Register Dji Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "all_drones_dji", "drone_type":"DjiMatrice"})
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
                        "name" : "all_drones_dji",
                        "type" : "DjiMatrice",
                        "topics" : [{"name": "topicNameDji", "type": "topicType"}],
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

    @timeout_decorator.timeout(TIMEOUT)
    def test_all_drones_available_mavros(self):
        # Register Mavros Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "all_drones_mavros", "drone_type":"Mavros"})
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
                        "name" : "all_drones_mavros",
                        "type" : "Mavros",
                        "topics" : [{"name": "topicNameMavros", "type": "topicType"}],
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

    @timeout_decorator.timeout(TIMEOUT)
    def test_query_topics_dji(self):
        # Register Dji Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "query_dji", "drone_type":"DjiMatrice"})
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
        self.assertFalse(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_query_topics_mavros(self):
        # Register Mavros Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "query_mavros", "drone_type":"Mavros"})
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
        self.assertFalse(result["success"])


class TestDjimatriceCreation(unittest.TestCase):

    @timeout_decorator.timeout(TIMEOUT)
    def test_register_drone(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "register_dji", "drone_type":"DjiMatrice"})
        result = service.call(request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_same_name(self):
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "same_name", "drone_type":"DjiMatrice"})
        result = service.call(request)
        self.assertTrue(result["success"])

        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "same_name", "drone_type":"DjiMatrice"})
        result = service.call(request)
        self.assertFalse(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_save_drone_topics(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "topics_dji", "drone_type":"DjiMatrice"})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": result["id"]})
        result = service.call(request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_shutdown_drone(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "shutdown_dji", "drone_type":"DjiMatrice"})
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

    @timeout_decorator.timeout(TIMEOUT)
    def test_register_drone(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "register_mavros", "drone_type":"Mavros"})
        result = service.call(request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_same_name(self):
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "same_name_mavros", "drone_type":"Mavros"})
        result = service.call(request)
        self.assertTrue(result["success"])

        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "same_name_mavros", "drone_type":"Mavros"})
        result = service.call(request)
        self.assertFalse(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_save_drone_topics(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "topics_mavros", "drone_type":"Mavros"})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": result["id"]})
        result = service.call(request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_shutdown_drone(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "shutdown_mavros", "drone_type":"Mavros"})
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


class TestDjimatriceControl(unittest.TestCase):

    @timeout_decorator.timeout(TIMEOUT)
    def test_start_mission(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "start_dji", "drone_type":"DjiMatrice"})
        result = service.call(request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Start Mission
        service = roslibpy.Service(client, 'isaacs_server/control_drone', 'isaacs_server/control_drone')
        request = roslibpy.ServiceRequest({"control_task": "start_mission", "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])
        self.assertEqual(result["id"], uid)

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_pause_mission(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "pause_dji", "drone_type":"DjiMatrice"})
        result = service.call(request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Pause Mission
        service = roslibpy.Service(client, 'isaacs_server/control_drone', 'isaacs_server/control_drone')
        request = roslibpy.ServiceRequest({"control_task": "pause_mission", "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])
        self.assertEqual(result["id"], uid)

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_resume_mission(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "resume_dji", "drone_type":"DjiMatrice"})
        result = service.call(request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Resume Mission
        service = roslibpy.Service(client, 'isaacs_server/control_drone', 'isaacs_server/control_drone')
        request = roslibpy.ServiceRequest({"control_task": "resume_mission", "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])
        self.assertEqual(result["id"], uid)

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_land_drone(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "land_dji", "drone_type":"DjiMatrice"})
        result = service.call(request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Land Drone
        service = roslibpy.Service(client, 'isaacs_server/control_drone', 'isaacs_server/control_drone')
        request = roslibpy.ServiceRequest({"control_task": "land_drone", "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])
        self.assertEqual(result["id"], uid)

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_fly_home(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "home_dji", "drone_type":"DjiMatrice"})
        result = service.call(request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Fly Home
        service = roslibpy.Service(client, 'isaacs_server/control_drone', 'isaacs_server/control_drone')
        request = roslibpy.ServiceRequest({"control_task": "fly_home", "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])
        self.assertEqual(result["id"], uid)

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

class TestMavrosControl(unittest.TestCase):

    @timeout_decorator.timeout(TIMEOUT)
    def test_start_mission(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "start_mavros", "drone_type":"Mavros"})
        result = service.call(request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Start Mission
        service = roslibpy.Service(client, 'isaacs_server/control_drone', 'isaacs_server/control_drone')
        request = roslibpy.ServiceRequest({"control_task": "start_mission", "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])
        self.assertEqual(result["id"], uid)

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_pause_mission(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "pause_mavros", "drone_type":"Mavros"})
        result = service.call(request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Pause Mission
        service = roslibpy.Service(client, 'isaacs_server/control_drone', 'isaacs_server/control_drone')
        request = roslibpy.ServiceRequest({"control_task": "pause_mission", "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])
        self.assertEqual(result["id"], uid)

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_resume_mission(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "resume_mavros", "drone_type":"Mavros"})
        result = service.call(request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Resume Mission
        service = roslibpy.Service(client, 'isaacs_server/control_drone', 'isaacs_server/control_drone')
        request = roslibpy.ServiceRequest({"control_task": "resume_mission", "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])
        self.assertEqual(result["id"], uid)

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_land_drone(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "land_mavros", "drone_type":"Mavros"})
        result = service.call(request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Land Drone
        service = roslibpy.Service(client, 'isaacs_server/control_drone', 'isaacs_server/control_drone')
        request = roslibpy.ServiceRequest({"control_task": "land_drone", "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])
        self.assertEqual(result["id"], uid)

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_fly_home(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "home_mavros", "drone_type":"Mavros"})
        result = service.call(request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

        # Fly Home
        service = roslibpy.Service(client, 'isaacs_server/control_drone', 'isaacs_server/control_drone')
        request = roslibpy.ServiceRequest({"control_task": "fly_home", "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])
        self.assertEqual(result["id"], uid)

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = service.call(request)
        self.assertTrue(result["success"])

if __name__ == '__main__':
    unittest.main()
