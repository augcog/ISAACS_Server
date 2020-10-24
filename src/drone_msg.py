class drone_msg:
    def __init__(self, id, name, type, topics, services):
    '''
    id = int with drone id
    name = string with name of drone
    type = string with drone type
    topics = list of topic_types (topics) published by drone
    services = list of topic_types (services) advertised by drone
    '''
        self.id = id
        self.name = name
        self.type = type
        self.topics = topics
        self.services = services
