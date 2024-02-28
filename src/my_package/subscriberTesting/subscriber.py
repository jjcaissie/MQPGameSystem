#!/usr/bin/python3
import rospy
from std_msgs.msg import String

def subscriberCallBack(data):
    rospy.loginfo(rospy.get_caller_id() + "Recieved: %s", data.data)


def listener():
    rospy.init_node('subscriberNode', anonymous=True)
    rospy.Subscriber("publisher", String, subscriberCallBack)
    rospy.spin()

#Run if run directly
if __name__ == '__main__':
    listener()
