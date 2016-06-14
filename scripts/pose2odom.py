#! /usr/bin/env python
# -*- coding:utf-8 -*-

# nav_msgs::Odometry -> geometry_msgs::Poseに変換

import rospy
# from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

print "Hello world"
cov_thresh=0
pub=rospy.Publisher('odometrised', Odometry, queue_size=10)

def callback(pose):
    # cov_east = navsatfix.position_covariance[0]
    # cov_north = navsatfix.position_covariance[4]
    # if cov_east>cov_thresh or cov_north>cov_thresh:
    #     return
    odom = Odometry()
    odom.pose.pose = pose.pose
    odom.header = pose.header
    pub.publish(odom)

def main():
    rospy.init_node('pose2odom')

    # global cov_thresh
    # cov_thresh = rospy.get_param("~cov_thresh", 10**2)
    rospy.Subscriber("ndt_pose", PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
