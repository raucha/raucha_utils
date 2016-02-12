#! /usr/bin/env python
# -*- coding:utf-8 -*-

# filter out NavSatFix messages have larger covariance than trethold as outlier

import rospy
from sensor_msgs.msg import NavSatFix

print "Hello world"
cov_thresh=0
pub=rospy.Publisher('navsatfix_out', NavSatFix, queue_size=10)

def callback(navsatfix):
    cov_east = navsatfix.position_covariance[0]
    cov_north = navsatfix.position_covariance[4]
    if cov_east>cov_thresh or cov_north>cov_thresh:
        return
    pub.publish(navsatfix)

def main():
    rospy.init_node('navsatfix_filter')

    global cov_thresh
    cov_thresh = rospy.get_param("~cov_thresh", 10**2)
    rospy.Subscriber("navsatfix_in", NavSatFix, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
