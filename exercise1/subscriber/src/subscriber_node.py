#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32

def callback(data, args):
    #start with opening args
    pub_result = args 
    
    try:
        #convert to float
        value = float(data.data)
        if value != 0.0:
            result = value / 0.15
            #publish to topic
            pub_result.publish(result)
            rospy.loginfo("I received: %s, Total: %f", data.data, result)
        else:
            rospy.loginfo("Error: Division by 0 occurs.")
    except ValueError:
        rospy.loginfo("Error: Invalid data format.")


def subscriber():
    rospy.init_node('subscriber', anonymous=True)
    #publish to result topic
    pub_result = rospy.Publisher('kthfsdv/result', Float32, queue_size=10)
    rospy.Subscriber("Aslanoglou", String, callback, callback_args=(pub_result))
    rospy.spin()


if __name__ == '__main__':
    subscriber()