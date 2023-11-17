#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publisher_node():
    rospy.init_node('publisher_node', anonymous=True)
    pub = rospy.Publisher('Aslanoglou', String, queue_size=10) 
    rate = rospy.Rate(20) #frequency=20hz
    k = 0
    n = 4 #given
    while not rospy.is_shutdown():
        k += n
        pub.publish(str(k)) 
        rate.sleep() #shut

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass