#!/usr/bin/env python

import rospy
from tf_monitor_msgs.srv import GetStampedTopicTf, GetStampedTopicTfRequest, SetStaticTf, SetStaticTfRequest
from geometry_msgs.msg import PoseStamped, TransformStamped


class PointCloudTransform(object):
    def __init__(self):
        # Basic Data
        self.map_initialized = False
        self.map_tf = None

        # Publishers
        self.orb_tf_pub = rospy.Publisher("/orb_slam2_mono/pose_tf", TransformStamped, queue_size=10)

        # Subscribers
        self.orb_pose_sub = rospy.Subscriber("/orb_slam2_mono/pose", PoseStamped, self.orb_pose_cb)
        self.unity_tf_sub = rospy.Subscriber("/path_camera/tfStamped", TransformStamped, self.unity_tf_cb)

        # wait for services to be ready
        rospy.wait_for_service("/tf_monitor/SetStaticTf")
        rospy.wait_for_service("/tf_monitor/RequestStampedTopicTf")

        # Services
        self.static_srv = rospy.ServiceProxy("/tf_monitor/SetStaticTf", SetStaticTf)
        self.tf_topic_srv = rospy.ServiceProxy("/tf_monitor/RequestStampedTopicTf", GetStampedTopicTf)


    def orb_pose_cb(self, msg):
        """
        :type msg: PoseStamped
        :param msg:
        :return:
        """
        if not self.map_initialized:
            self.map_initialized = True
            static_request = SetStaticTfRequest()
            static_request.tf = self.map_tf
            static_request.tf.header.frame_id = "Unity"
            static_request.tf.child_frame_id = "map"
            self.static_srv.call(static_request)
            rospy.loginfo("Requesting static")

            topic_request = GetStampedTopicTfRequest()
            topic_request.out_frame = "Child"
            topic_request.topic_name = "/orb_slam2_mono/map_points"
            topic_request.out_topic_name_recommendation = "/unity/map_points"
            self.tf_topic_srv.call(topic_request)
            rospy.loginfo("Requesting Topic Transformation for PointCloud2")
        else:
            tf_msg = TransformStamped()
            tf_msg.header = msg.header
            tf_msg.header.frame_id = "map"
            tf_msg.child_frame_id = "camera_link"
            tf_msg.transform.translation.x = msg.pose.position.x
            tf_msg.transform.translation.y = msg.pose.position.y
            tf_msg.transform.translation.z = msg.pose.position.z
            tf_msg.transform.rotation.x = msg.pose.orientation.x
            tf_msg.transform.rotation.y = msg.pose.orientation.y
            tf_msg.transform.rotation.z = msg.pose.orientation.z
            tf_msg.transform.rotation.w = msg.pose.orientation.w
            self.orb_tf_pub.publish(tf_msg)

    def unity_tf_cb(self, msg):
        self.map_tf = msg


if __name__ == "__main__":
    rospy.init_node("PCManager", anonymous=True)
    pc = PointCloudTransform()
    rospy.spin()
