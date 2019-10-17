#!/usr/bin/env python

import sys
import unittest
import rospy
import rostest
from tf_monitor_msgs.srv import GetTf, GetTfResponse, GetTfRequest

PKG = 'tf_monitor'
NAME = 'request_tf_test'


class RequestTf(unittest.TestCase):
    def __init__(self, *args):
        super(RequestTf, self).__init__(*args)

    def test_a_single_request(self):
        rospy.wait_for_service("tf_monitor/RequestTf")
        service = rospy.ServiceProxy("tf_monitor/RequestTf/", GetTf)
        request = GetTfRequest()
        request.in_frame = "robot"
        request.out_frame = "holospace"
        request.wait = 0

        rospy.sleep(5)
        response = service.call(request)  # type:GetTfResponse

        self.assertAlmostEqual(response.tf.transform.translation.y, 2, 2)
        self.assertAlmostEqual(response.tf.transform.rotation.z, -0.707106818813, 2)
        self.assertAlmostEqual(response.tf.transform.rotation.w, 0.707106818813, 2)


if __name__ == "__main__":
    rostest.rosrun(PKG, NAME, RequestTf, sys.argv)
