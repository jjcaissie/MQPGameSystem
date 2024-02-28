#!/usr/bin/python3
import rospy
import main
from std_msgs.msg import String

def __publishToNode__():
    pub = rospy.Publisher('publisher', String, queue_size = 10)
    rospy.init_node('publish_node', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    	publishString = ()
    	rospy.loginfo("Data is being sent")
    	pub.publish(publishString)
    	rate.sleep()

if __name__ == '__main__':
    try:
        __publishToNode__()
    except rospy.ROSInterruptException:
        pass
