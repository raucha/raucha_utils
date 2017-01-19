#! /usr/bin/env python
# -*- coding:utf-8 -*-

import unittest
import rosmsg_utils as ru
from std_msgs.msg import Float32
from std_msgs.msg import Header
from std_msgs.msg import Int16MultiArray
import rospy

d2m = ru.dict2msg
m2d = ru.msg2dict
isEqu = ru.isEqualRosMsg


class MyTest(unittest.TestCase):

    def test_conv_float(self):
        a = Float32()
        b = d2m(Float32(), m2d(a))
        self.assertTrue(isEqu(a, b))
        a.data = 1
        self.assertFalse(isEqu(a, b))
        b = d2m(Float32(), m2d(a))
        self.assertTrue(isEqu(a, b))

    def test_conv_header(self):
        func = lambda x: d2m(Header(), m2d(x))
        print
        a = Header()
        self.assertTrue(isEqu(a, func(a)))
        a2 = Header()
        a2.frame_id = "fix"
        self.assertFalse(isEqu(a2, func(a)))
        self.assertTrue(isEqu(a2, func(a2)))
        a3 = Header()
        a3.stamp = rospy.Time(10)
        self.assertFalse(isEqu(a3, func(a)))
        self.assertTrue(isEqu(a3, func(a3)))

    def test_conv_array(self):
        func = lambda x: d2m(Int16MultiArray(), m2d(x))
        a = Int16MultiArray()
        self.assertTrue(isEqu(a, func(a)))
        a2 = Int16MultiArray()
        a2.data.append(1)
        self.assertFalse(isEqu(a, func(a2)))
        self.assertTrue(isEqu(a2, func(a2)))
        a2.data.append(2)
        a2.data.append(3)
        self.assertTrue(isEqu(a2, func(a2)))


if __name__ == "__main__":
    unittest.main()
