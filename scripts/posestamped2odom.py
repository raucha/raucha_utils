#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
# from sensor_msgs.msg import NavSatFix
# from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

print "Hello world"
pub=rospy.Publisher('odom_out', Odometry, queue_size=10)

def callback(input):
    output = Odometry()
    output.header = input.header
    output.pose.pose = input.pose
    pub.publish(output)

def main():
    rospy.init_node('posestamped2odom')

    # cov_thresh = rospy.get_param("~cov_thresh", 10**2)
    rospy.Subscriber("posestamped_in", PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
