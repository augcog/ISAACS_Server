#!/usr/bin/env python

'''
This is the example creating publisher for topics.
The script is tested with both roslibpy and rospy and both work
with the exception that rospy needs to be on same network.
Found at: https://roslibpy.readthedocs.io/en/latest/examples.html
'''

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
        