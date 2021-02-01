from drone import Drone

mavros_drone = Drone.create('mavros_drone', 'MavrosDrone')
mavros_drone.upload_mission([
    {'frame': 3, 'command': 16, 'is_current': False, 'autocontinue': True, 'param1': 0, 'param2': 0, 'param3': 0, 'x_lat': -35.364652, 'y_long': 149.163501, 'z_alt': 20},
    {'frame': 3, 'command': 16, 'is_current': False, 'autocontinue': True, 'param1': 0, 'param2': 0, 'param3': 0, 'x_lat': -35.365361, 'y_long': 149.163995, 'z_alt': 20}])