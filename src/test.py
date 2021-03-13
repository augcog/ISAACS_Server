import unittest
import roslibpy
import roslibpy.actionlib
import timeout_decorator
import constants

'''
Make sure to restart operator.py before running make
Make sure to make each drone_name unique
'''

TIMEOUT = 30

client = roslibpy.Ros(host=constants.IP_ADDRESS, port=9090)
#client = roslibpy.Ros(host='0.0.0.0', port=9090)

def wrapped_service_call(service, request):
    result = None
    attempts = 0
    while (result == None) and (attempts <= 5):
        if (attempts > 0):
            print("Took ", attempts, " attempt(s)")
        try:
            result = service.call(request, timeout=3)
        except:
            result = None
            attempts += 1
    return result

def serverReset():
    service = roslibpy.Service(client, 'isaacs_server/reset', 'isaacs_server/reset')
    request = roslibpy.ServiceRequest({})
    result = wrapped_service_call(service, request)

class ActionClientWorkaround(roslibpy.actionlib.ActionClient):
    def setCustomTopics(self):
        # Sets topic to custom action topics
        self.feedback_listener = roslibpy.Topic(self.ros, self.server_name + '/Actionfeedback', self.action_name + 'ActionFeedback')
        self.result_listener = roslibpy.Topic(self.ros, self.server_name + '/Actionresult', self.action_name + 'ActionResult')
        self.goal_topic = roslibpy.Topic(self.ros, self.server_name + '/Actiongoal', self.action_name + 'ActionGoal')
        # Advertise the goal and cancel topics
        self.goal_topic.advertise()
        # Subscribe to the feedback topic
        if not self.omit_feedback:
            self.feedback_listener.subscribe(self._on_feedback_message)
        # Subscribe to the result topic
        if not self.omit_result:
            self.result_listener.subscribe(self._on_result_message)

#Returns a navsatfix given a lat, long, alt
def navsatfix(lat, long, alt):
    ret = {}
    ret["header"] = None
    ret["stats"] = None
    ret["latitude"] = lat
    ret["longitude"] = long
    ret["altitude"] = alt
    ret["position_covariance"] = [0,0,0,0,0,0,0,0,0]
    ret["position_covariance_type"] = 0
    return ret
    #return roslibpy.message(ret)


class TestVRConnection(unittest.TestCase):

    @timeout_decorator.timeout(TIMEOUT)
    def test_all_drones_available_dji(self):
        # Register Dji Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "all_drones_dji", "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Dji Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # All Drones Available
        service = roslibpy.Service(client, 'isaacs_server/all_drones_available', 'isaacs_server/all_drones_available')
        request = roslibpy.ServiceRequest({})
        result = wrapped_service_call(service, request)
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
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # All Drones Available
        service = roslibpy.Service(client, 'isaacs_server/all_drones_available', 'isaacs_server/all_drones_available')
        request = roslibpy.ServiceRequest({})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        self.assertNotIn(test_drone, result['drones_available'])

    @timeout_decorator.timeout(TIMEOUT)
    def test_all_drones_available_mavros(self):
        # Register Mavros Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "all_drones_mavros", "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Mavros Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # All Drones Available
        service = roslibpy.Service(client, 'isaacs_server/all_drones_available', 'isaacs_server/all_drones_available')
        request = roslibpy.ServiceRequest({})
        result = wrapped_service_call(service, request)
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
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # All Drones Available
        service = roslibpy.Service(client, 'isaacs_server/all_drones_available', 'isaacs_server/all_drones_available')
        request = roslibpy.ServiceRequest({})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        self.assertNotIn(test_drone, result['drones_available'])

    @timeout_decorator.timeout(TIMEOUT)
    def test_query_topics_dji(self):
        # Register Dji Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "query_dji", "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Dji Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Query Topics
        service = roslibpy.Service(client, 'isaacs_server/query_topics', 'isaacs_server/query_topics')
        request = roslibpy.ServiceRequest({"id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        self.assertIn({"name": "topicNameDji", "type": "topicType"}, result['all_topics'])

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Query Topics
        service = roslibpy.Service(client, 'isaacs_server/query_topics', 'isaacs_server/query_topics')
        request = roslibpy.ServiceRequest({"id": uid})
        result = wrapped_service_call(service, request)
        self.assertFalse(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_query_topics_mavros(self):
        # Register Mavros Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "query_mavros", "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Mavros Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Query Topics
        service = roslibpy.Service(client, 'isaacs_server/query_topics', 'isaacs_server/query_topics')
        request = roslibpy.ServiceRequest({"id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        self.assertIn({"name": "topicNameMavros", "type": "topicType"}, result['all_topics'])

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Query Topics
        service = roslibpy.Service(client, 'isaacs_server/query_topics', 'isaacs_server/query_topics')
        request = roslibpy.ServiceRequest({"id": uid})
        result = wrapped_service_call(service, request)
        self.assertFalse(result["success"])


class TestDjimatriceCreation(unittest.TestCase):

    @timeout_decorator.timeout(TIMEOUT)
    def test_register_drone(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "register_dji", "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_save_drone_topics(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "topics_dji", "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": result["id"]})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_shutdown_drone(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "shutdown_dji", "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])


class TestMavrosCreation(unittest.TestCase):

    @timeout_decorator.timeout(TIMEOUT)
    def test_register_drone(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "register_mavros", "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_save_drone_topics(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "topics_mavros", "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": result["id"]})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_shutdown_drone(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "shutdown_mavros", "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])


class TestDjimatriceControl(unittest.TestCase):

    @timeout_decorator.timeout(TIMEOUT)
    def test_start_mission(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "start_dji", "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Start Mission
        action_client = ActionClientWorkaround(client,"isaacs_server/control_drone",'isaacs_server/control_drone')
        action_client.setCustomTopics()
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"start_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_pause_mission(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "pause_dji", "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Pause Mission
        action_client = ActionClientWorkaround(client,"isaacs_server/control_drone",'isaacs_server/control_drone')
        action_client.setCustomTopics()
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"pause_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_resume_mission(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "resume_dji", "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Resume Mission
        action_client = ActionClientWorkaround(client,"isaacs_server/control_drone",'isaacs_server/control_drone')
        action_client.setCustomTopics()
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"resume_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_stop_mission(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "stop_dji", "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Stop Mission
        action_client = ActionClientWorkaround(client,"isaacs_server/control_drone",'isaacs_server/control_drone')
        action_client.setCustomTopics()
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"stop_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_land_drone(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "land_dji", "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Land Drone
        action_client = ActionClientWorkaround(client,"isaacs_server/control_drone",'isaacs_server/control_drone')
        action_client.setCustomTopics()
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"land_drone"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_fly_home(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "home_dji", "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Fly Home
        action_client = ActionClientWorkaround(client,"isaacs_server/control_drone",'isaacs_server/control_drone')
        action_client.setCustomTopics()
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"fly_home"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

class TestMavrosControl(unittest.TestCase):

    @timeout_decorator.timeout(TIMEOUT)
    def test_start_mission(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "start_mavros", "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Start Mission
        action_client = ActionClientWorkaround(client,"isaacs_server/control_drone",'isaacs_server/control_drone')
        action_client.setCustomTopics()
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"start_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_pause_mission(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "pause_mavros", "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Pause Mission
        action_client = ActionClientWorkaround(client,"isaacs_server/control_drone",'isaacs_server/control_drone')
        action_client.setCustomTopics()
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"pause_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_resume_mission(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "resume_mavros", "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Resume Mission
        action_client = ActionClientWorkaround(client,"isaacs_server/control_drone",'isaacs_server/control_drone')
        action_client.setCustomTopics()
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"resume_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_stop_mission(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "stop_mavros", "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Stop Mission
        action_client = ActionClientWorkaround(client,"isaacs_server/control_drone",'isaacs_server/control_drone')
        action_client.setCustomTopics()
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"stop_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_land_drone(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "land_mavros", "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Land Drone
        action_client = ActionClientWorkaround(client,"isaacs_server/control_drone",'isaacs_server/control_drone')
        action_client.setCustomTopics()
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"land_drone"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_fly_home(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "home_mavros", "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Fly Home
        action_client = ActionClientWorkaround(client,"isaacs_server/control_drone",'isaacs_server/control_drone')
        action_client.setCustomTopics()
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"fly_home"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    def test_upload_mission(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/register_drone')
        request = roslibpy.ServiceRequest({'drone_name': "upload_mavros", "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Upload_mission
        action_client = ActionClientWorkaround(client,"isaacs_server/upload_mission",'isaacs_server/upload_mission')
        action_client.setCustomTopics()
        waypoints = [navsatfix(0,0,0), navsatfix(1,1,1)]
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "waypoints":waypoints}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/type_to_topic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

class TestIsolatedControl(unittest.TestCase):
    def test_isolated_fly_home(self):
        # Fly Home
        if not client.is_connected:
            client.run()
        action_client = ActionClientWorkaround(client,"isaacs_server/control_drone",'isaacs_server/control_drone')
        action_client.setCustomTopics()
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': 1, "control_task":"fly_home"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

    def test_isolated_land_drone(self):
        # Land Drone
        if not client.is_connected:
            client.run()
        action_client = ActionClientWorkaround(client,"isaacs_server/control_drone",'isaacs_server/control_drone')
        action_client.setCustomTopics()
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': 1, "control_task":"land_drone"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

    def test_isolated_pause_mission(self):
        # Pause Mission
        if not client.is_connected:
            client.run()
        action_client = ActionClientWorkaround(client,"isaacs_server/control_drone",'isaacs_server/control_drone')
        action_client.setCustomTopics()
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': 1, "control_task":"pause_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

    def test_isolated_stop_mission(self):
        # Stop Mission
        if not client.is_connected:
            client.run()
        action_client = ActionClientWorkaround(client,"isaacs_server/control_drone",'isaacs_server/control_drone')
        action_client.setCustomTopics()
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': 1, "control_task":"stop_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

    def test_isolated_resume_mission(self):
        # Resume Mission
        if not client.is_connected:
            client.run()
        action_client = ActionClientWorkaround(client,"isaacs_server/control_drone",'isaacs_server/control_drone')
        action_client.setCustomTopics()
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': 1, "control_task":"resume_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

    def test_isolated_start_mission(self):
        # Start Mission
        if not client.is_connected:
            client.run()
        action_client = ActionClientWorkaround(client,"isaacs_server/control_drone",'isaacs_server/control_drone')
        action_client.setCustomTopics()
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': 1, "control_task":"start_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        action_client.dispose()

if __name__ == '__main__':
    unittest.main()
