class sensor_msg:
    def __init__(self, id, name, type, topics, services, parent_drone_id):
    '''
    id = int with sensor id
    name = string with name of sensor
    type = string with drone sensor
    topics = list of topic_types (topics) published by sensor
    services = list of topic_types (services) advertised by sensor
    parent_drone_id = int with id of parent drone
    '''
        self.id = id
        self.name = name
        self.type = type
        self.topics = topics
        self.services = services
        self.parent_drone_id = parent_drone_id
