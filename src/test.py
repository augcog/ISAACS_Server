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
    service = roslibpy.Service(client, 'isaacs_server/reset', 'isaacs_server/Reset')
    request = roslibpy.ServiceRequest({})
    result = wrapped_service_call(service, request)

#Returns a navsatfix given a lat, long, alt
def navsatfix(lat, long, alt):
    ret = {}
    ret["header"] = {'seq': 885, 'stamp': {'secs' : 1552399290, 'nsecs': 267234086}, 'frame_id': "/wgs84"}
    ret["status"] = {"status": 0, "service": 1}
    ret["latitude"] = lat
    ret["longitude"] = long
    ret["altitude"] = alt
    ret["position_covariance"] = [0,0,0,0,0,0,0,0,0]
    ret["position_covariance_type"] = 0
    return ret


class TestVRConnection(unittest.TestCase):

    @timeout_decorator.timeout(TIMEOUT)
    def test_all_drones_available_dji(self):
        # Register Dji Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/RegisterDrone')
        request = roslibpy.ServiceRequest({'drone_name': "all_drones_dji", "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Dji Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # All Drones Available
        service = roslibpy.Service(client, 'isaacs_server/all_drones_available', 'isaacs_server/AllDronesAvailable')
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
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # All Drones Available
        service = roslibpy.Service(client, 'isaacs_server/all_drones_available', 'isaacs_server/AllDronesAvailable')
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
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/RegisterDrone')
        request = roslibpy.ServiceRequest({'drone_name': "all_drones_mavros", "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Mavros Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # All Drones Available
        service = roslibpy.Service(client, 'isaacs_server/all_drones_available', 'isaacs_server/AllDronesAvailable')
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
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # All Drones Available
        service = roslibpy.Service(client, 'isaacs_server/all_drones_available', 'isaacs_server/AllDronesAvailable')
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
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/RegisterDrone')
        request = roslibpy.ServiceRequest({'drone_name': "query_dji", "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Dji Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Query Topics
        service = roslibpy.Service(client, 'isaacs_server/query_topics', 'isaacs_server/QueryTopics')
        request = roslibpy.ServiceRequest({"id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        self.assertIn({"name": "topicNameDji", "type": "topicType"}, result['all_topics'])

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Query Topics
        service = roslibpy.Service(client, 'isaacs_server/query_topics', 'isaacs_server/QueryTopics')
        request = roslibpy.ServiceRequest({"id": uid})
        result = wrapped_service_call(service, request)
        self.assertFalse(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_query_topics_mavros(self):
        # Register Mavros Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/RegisterDrone')
        request = roslibpy.ServiceRequest({'drone_name': "query_mavros", "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Mavros Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Query Topics
        service = roslibpy.Service(client, 'isaacs_server/query_topics', 'isaacs_server/QueryTopics')
        request = roslibpy.ServiceRequest({"id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        self.assertIn({"name": "topicNameMavros", "type": "topicType"}, result['all_topics'])

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Query Topics
        service = roslibpy.Service(client, 'isaacs_server/query_topics', 'isaacs_server/QueryTopics')
        request = roslibpy.ServiceRequest({"id": uid})
        result = wrapped_service_call(service, request)
        self.assertFalse(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_query_topics_depth_camera(self):
        # Register Dji Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/RegisterDrone')
        request = roslibpy.ServiceRequest({'drone_name': "query_dji_camera", "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Dji Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Register Depth Camera Sesnor
        service = roslibpy.Service(client, 'isaacs_server/register_sensor', 'isaacs_server/RegisterSensor')
        request = roslibpy.ServiceRequest({'sensor_name': "query_depth_camera", "sensor_type":"Depth Camera", "parent_drone_name":"query_dji_camera"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        suid = result["id"]

        # Save Sensor Topics
        spublishes = [{"name": "depthCamera", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_sensor_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": spublishes, "id": suid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Query Topics
        service = roslibpy.Service(client, 'isaacs_server/query_topics', 'isaacs_server/QueryTopics')
        request = roslibpy.ServiceRequest({"id": suid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        self.assertIn({"name": "depthCamera", "type": "topicType"}, result['all_topics'])

        # Shutdown Sensor
        service = roslibpy.Service(client, 'isaacs_server/shutdown_sensor', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": spublishes, "id": suid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])


class TestDjimatriceCreation(unittest.TestCase):

    @timeout_decorator.timeout(TIMEOUT)
    def test_register_drone(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/RegisterDrone')
        request = roslibpy.ServiceRequest({'drone_name': "register_dji", "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_save_drone_topics(self):
        # Register Drone
        if not client.is_connected:
            client.run()

        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/RegisterDrone')
        request = roslibpy.ServiceRequest({'drone_name': "topics_dji", "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": result["id"]})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_shutdown_drone(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/RegisterDrone')
        request = roslibpy.ServiceRequest({'drone_name': "shutdown_dji", "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/TypeToTopic')
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
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/RegisterDrone')
        request = roslibpy.ServiceRequest({'drone_name': "register_mavros", "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_save_drone_topics(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/RegisterDrone')
        request = roslibpy.ServiceRequest({'drone_name': "topics_mavros", "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": result["id"]})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_shutdown_drone(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/RegisterDrone')
        request = roslibpy.ServiceRequest({'drone_name': "shutdown_mavros", "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])


class TestDjimatriceControl(unittest.TestCase):
    # Helper Method that rejisters a DJI drone with drone_name and returns its UID if successful
    @timeout_decorator.timeout(TIMEOUT)
    def register_DJI_drone(self, drone_name):
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/RegisterDrone')
        request = roslibpy.ServiceRequest({'drone_name': drone_name, "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]
        return uid

    # Helper Method that shutsdown DJI drone with UID
    def shutdown_DJI_drone(self, publishes, uid):
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_start_mission(self):
        # Register Drone
        uid = self.register_DJI_drone("start_dji")

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Start Mission
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/control_drone",'isaacs_server/ControlDroneAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"start_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])

        # Shutdown Drone
        self.shutdown_DJI_drone(publishes, uid)

    @timeout_decorator.timeout(TIMEOUT)
    def test_pause_mission(self):
        # Register Drone
        uid = self.register_DJI_drone("pause_dji")

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Pause Mission
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/control_drone",'isaacs_server/ControlDroneAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"pause_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])

        # Shutdown Drone
        self.shutdown_DJI_drone(publishes, uid)

    @timeout_decorator.timeout(TIMEOUT)
    def test_resume_mission(self):
        # Register Drone
        uid = self.register_DJI_drone("resume_dji")

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Resume Mission
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/control_drone",'isaacs_server/ControlDroneAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"resume_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])

        # Shutdown Drone
        self.shutdown_DJI_drone(publishes, uid)

    @timeout_decorator.timeout(TIMEOUT)
    def test_stop_mission(self):
        # Register Drone
        uid = self.register_DJI_drone("stop_dji")

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Stop Mission
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/control_drone",'isaacs_server/ControlDroneAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"stop_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])

        # Shutdown Drone
        self.shutdown_DJI_drone(publishes, uid)

    @timeout_decorator.timeout(TIMEOUT)
    def test_land_drone(self):
        # Register Drone
        uid = self.register_DJI_drone("land_dji")

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Land Drone
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/control_drone",'isaacs_server/ControlDroneAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"land_drone"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])

        # Shutdown Drone
        self.shutdown_DJI_drone(publishes, uid)

    @timeout_decorator.timeout(TIMEOUT)
    def test_fly_home(self):
        # Register Drone
        uid = self.register_DJI_drone("home_dji")

        # Save Topics
        publishes = [{"name": "topicNameDji", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Fly Home
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/control_drone",'isaacs_server/ControlDroneAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"fly_home"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])

        # Shutdown Drone
        self.shutdown_DJI_drone(publishes, uid)

    @timeout_decorator.timeout(TIMEOUT)
    def test_upload_mission(self):
        # Register Drone
        uid = self.register_DJI_drone("upload_dji")

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Upload_mission
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/upload_mission",'isaacs_server/UploadMissionAction')
        waypoints = [navsatfix(-35.362881,149.165222,0), navsatfix(-35.362881,149.163501,40)]
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "waypoints":waypoints}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])

        # Shutdown Drone
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_get_set_speed(self):
        # Register Drone
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/RegisterDrone')
        request = roslibpy.ServiceRequest({'drone_name': "speed_dji", "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # get speed
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/get_speed",'isaacs_server/GetSpeedAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        self.assertTrue(uid == result["id"])
        speed = result["speed"]
        newSpeed = speed + 1

        # set speed
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/set_speed",'isaacs_server/SetSpeedAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "speed":newSpeed}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        self.assertTrue(uid == result["id"])

        # get speed
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/get_speed",'isaacs_server/GetSpeedAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])
        self.assertTrue(uid == result["id"])
        self.assertTrue(newSpeed == result["speed"])

        # Shutdown Drone
        self.shutdown_DJI_drone(publishes, uid)

class TestMavrosControl(unittest.TestCase):

    # Helper Method that rejisters a mavros drone with drone_name and returns its UID if successful
    @timeout_decorator.timeout(TIMEOUT)
    def register_mavros_drone(self, drone_name):
        if not client.is_connected:
            client.run()
        serverReset()
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/RegisterDrone')
        request = roslibpy.ServiceRequest({'drone_name': drone_name, "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]
        return uid

    # Helper Method that shutsdown mavros drone with UID
    def shutdown_mavros_drone(self, publishes, uid):
        service = roslibpy.Service(client, 'isaacs_server/shutdown_drone', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

    @timeout_decorator.timeout(TIMEOUT)
    def test_start_mission(self):
        # Register Drone
        uid = self.register_mavros_drone("start_mavros")

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Start Mission
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/control_drone",'isaacs_server/ControlDroneAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"start_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])

        # Shutdown Drone
        self.shutdown_mavros_drone(publishes, uid)

    @timeout_decorator.timeout(TIMEOUT)
    def test_pause_mission(self):
        # Register Drone
        uid = self.register_mavros_drone("pause_mavros")

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Pause Mission
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/control_drone",'isaacs_server/ControlDroneAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"pause_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])

        # Shutdown Drone
        self.shutdown_mavros_drone(publishes, uid)

    @timeout_decorator.timeout(TIMEOUT)
    def test_resume_mission(self):
        # Register Drone
        uid = self.register_mavros_drone("resume_mavros")

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Resume Mission
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/control_drone",'isaacs_server/ControlDroneAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"resume_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])

        # Shutdown Drone
        self.shutdown_mavros_drone(publishes, uid)

    @timeout_decorator.timeout(TIMEOUT)
    def test_stop_mission(self):
        # Register Drone
        uid = self.register_mavros_drone("stop_mavros")

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Stop Mission
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/control_drone",'isaacs_server/ControlDroneAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"stop_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])

        # Shutdown Drone
        self.shutdown_mavros_drone(publishes, uid)

    @timeout_decorator.timeout(TIMEOUT)
    def test_land_drone(self):
        # Register Drone
        uid = self.register_mavros_drone("land_mavros")

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Land Drone
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/control_drone",'isaacs_server/ControlDroneAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"land_drone"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])

        # Shutdown Drone
        self.shutdown_mavros_drone(publishes, uid)

    @timeout_decorator.timeout(TIMEOUT)
    def test_fly_home(self):
        # Register Drone
        uid = self.register_mavros_drone("home_mavros")

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Fly Home
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/control_drone",'isaacs_server/ControlDroneAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "control_task":"fly_home"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])

        # Shutdown Drone
        self.shutdown_mavros_drone(publishes, uid)

    def test_upload_mission(self):
        # Register Drone
        uid = self.register_mavros_drone("upload_mavros")

        # Save Topics
        publishes = [{"name": "topicNameMavros", "type": "topicType"}]
        service = roslibpy.Service(client, 'isaacs_server/save_drone_topics', 'isaacs_server/TypeToTopic')
        request = roslibpy.ServiceRequest({"publishes": publishes, "id": uid})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])

        # Upload_mission
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/upload_mission",'isaacs_server/UploadMissionAction')
        waypoints = [navsatfix(-35.362881,149.165222,0), navsatfix(-35.362881,149.163501,40)]
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': uid, "waypoints":waypoints}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])

        # Shutdown Drone
        self.shutdown_mavros_drone(publishes, uid)

class TestIsolatedControl(unittest.TestCase):

    @timeout_decorator.timeout(TIMEOUT)
    def register_mavros_drone(self, drone_name):
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/RegisterDrone')
        request = roslibpy.ServiceRequest({'drone_name': drone_name, "drone_type":"Mavros"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]
        return uid

    @timeout_decorator.timeout(TIMEOUT)
    def register_DJI_drone(self, drone_name):
        service = roslibpy.Service(client, 'isaacs_server/register_drone', 'isaacs_server/RegisterDrone')
        request = roslibpy.ServiceRequest({'drone_name': drone_name, "drone_type":"DjiMatrice"})
        result = wrapped_service_call(service, request)
        self.assertTrue(result["success"])
        uid = result["id"]
        return uid

    def test_isolated_register(self):
        if not client.is_connected:
            client.run()
        serverReset()
        # Register Drone
        uid = self.register_DJI_drone("DJI1")
        # Register Drone
        uid2 = self.register_DJI_drone("DJI2")


    def test_isolated_fly_home(self):
        # Fly Home
        if not client.is_connected:
            client.run()
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/control_drone",'isaacs_server/ControlDroneAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': 3, "control_task":"fly_home"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(30)
        self.assertTrue(result["success"])

    def test_isolated_land_drone(self):
        # Land Drone
        if not client.is_connected:
            client.run()
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/control_drone",'isaacs_server/ControlDroneAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': 3, "control_task":"land_drone"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(30)
        self.assertTrue(result["success"])

    def test_isolated_pause_mission(self):
        # Pause Mission
        if not client.is_connected:
            client.run()
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/control_drone",'isaacs_server/ControlDroneAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': 3, "control_task":"pause_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(30)
        self.assertTrue(result["success"])

    def test_isolated_stop_mission(self):
        # Stop Mission
        if not client.is_connected:
            client.run()
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/control_drone",'isaacs_server/ControlDroneAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': 3, "control_task":"stop_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(30)
        self.assertTrue(result["success"])

    def test_isolated_resume_mission(self):
        # Resume Mission
        if not client.is_connected:
            client.run()
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/control_drone",'isaacs_server/ControlDroneAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': 3, "control_task":"resume_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(30)
        self.assertTrue(result["success"])

    def test_isolated_start_mission(self):
        # Start Mission
        if not client.is_connected:
            client.run()
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/control_drone",'isaacs_server/ControlDroneAction')
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': 1, "control_task":"start_mission"}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(30)
        self.assertTrue(result["success"])

    def test_isolated_upload_mission(self):
        # Start Mission
        if not client.is_connected:
            client.run()
        action_client = roslibpy.actionlib.ActionClient(client,"isaacs_server/upload_mission",'isaacs_server/UploadMissionAction')
        waypoints = [navsatfix(-35.362881,149.165222,0), navsatfix(-35.362881,149.163501,40)]
        goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message({'id': 1, "waypoints":waypoints}))
        goal.on('feedback', lambda f: print(f['progress']))
        goal.send()
        result = goal.wait(10)
        self.assertTrue(result["success"])

if __name__ == '__main__':
    unittest.main()
