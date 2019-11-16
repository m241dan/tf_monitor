#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped

class DataCap(object):
    def __init__(self):
        self.true_sub = None
        self.orb_sub = rospy.Subscriber("/orb_slam2_mono/pose", PoseStamped, self.orb_pose_cb)

        self.true_prev = None
        self.orb_prev = None

        self.true_collection = []
        self.orb_collection = []

    def orb_pose_cb(self, msg):
        """
        :type msg: PoseStamped
        :param msg:
        :return:
        """
        if self.true_sub == None:
            self.true_sub = rospy.Subscriber("/path_camera/tfStamped", TransformStamped, self.unity_tf_cb)
        self.orb_collection.append((msg.header.stamp, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

    def unity_tf_cb(self, msg):
        """
        :type msg: TransformStamped
        :param msg:
        :return:
        """
        self.true_collection.append((msg.header.stamp, msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z))


if __name__ == "__main__":
    rospy.init_node("DataCap", anonymous=True)
    dc = DataCap()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

    with open("true.csv", "w") as f:
        for x in dc.true_collection:
            f.write("%s,%f,%f,%f\n" % x)
    with open("orb.csv", "w") as f:
        for x in dc.orb_collection:
            f.write("%s,%f,%f,%f\n" % x)
