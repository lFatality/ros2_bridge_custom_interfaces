#!/usr/bin/env python
# license removed for brevity
import rospy
from custom_msg_ros1.msg import CustomMessage

def talker():
    pub = rospy.Publisher('chatter', CustomMessage, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = CustomMessage()
        msg.custom_value = 1.22
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass 
