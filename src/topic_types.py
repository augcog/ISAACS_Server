class topic_types():
    def __init__(self, JSON):
        '''
        name = string with name of topic
        type = string with type of topic
        '''
        self.name = JSON.name
        self.type = JSON.type
