'''uint8 COVARIANCE_TYPE_UNKNOWN=0
uint8 COVARIANCE_TYPE_APPROXIMATED=1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
uint8 COVARIANCE_TYPE_KNOWN=3
std_msgs/Header header
sensor_msgs/NavSatStatus status
float64 latitude
float64 longitude
float64 altitude
float64[9] position_covariance
uint8 position_covariance_type

Header
uint32 seq
time stamp
string frame_id

NavSatStatus
int8 status
uint16 service

'x_lat': -35.364652, 'y_long': 149.163501, 'z_alt': 20
'x_lat': -35.365361, 'y_long': 149.163995, 'z_alt': 20'''


message = dict()
message["position_covariance_type"] = 0
message["position_covariance"] = 0
message["latitude"] = -35.364652
message["longitude"] = 149.163501
message["altitude"] = 20
message["position_covariance"] = [0, 0, 0, 0, 0, 0, 0, 0, 0]

header = dict()
header["seq"] = 1
header["frame_id"] = "1"
header["time"] = None
message["header"] = header

status = dict()
status["status"] = 0
status["service"] = 1
message["status"] = status


message2 = dict()
message2["position_covariance_type"] = 0
message2["position_covariance"] = 0
message2["latitude"] = -35.364652
message2["longitude"] = 149.164531
message2["altitude"] = 20
message2["position_covariance"] = [0, 0, 0, 0, 0, 0, 0, 0, 0]
message2["status"] = status
message2["header"] = header


to_send = [message, message2]
