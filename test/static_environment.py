#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
from rospy.timer import TimerEvent
from tf.transformations import translation_matrix, quaternion_matrix, concatenate_matrices, inverse_matrix, \
    translation_from_matrix, quaternion_from_matrix


def invert_transform(msg):
    """
    The purpose of this is to invert a TransformStamped message.
    :type msg: TransformStamped
    :param msg:
    :return: TransformStamped that has been inverted
    """
    trans_mat                   = translation_matrix([msg.transform.translation.x,
                                                      msg.transform.translation.y,
                                                      msg.transform.translation.z])
    quat_mat                    = quaternion_matrix([msg.transform.rotation.x,
                                                     msg.transform.rotation.y,
                                                     msg.transform.rotation.z,
                                                     msg.transform.rotation.w])
    homogenous_transformation   = concatenate_matrices(trans_mat, quat_mat)
    inverse_transformation      = inverse_matrix(homogenous_transformation)
    translation                 = translation_from_matrix(inverse_transformation)
    quaternion                  = quaternion_from_matrix(inverse_transformation)
    t                           = TransformStamped()
    t.header                    = msg.header
    t.child_frame_id            = msg.child_frame_id
    t.transform.translation.x   = translation[0]
    t.transform.translation.y   = translation[1]
    t.transform.translation.z   = translation[2]
    t.transform.rotation.x      = quaternion[0]
    t.transform.rotation.y      = quaternion[1]
    t.transform.rotation.z      = quaternion[2]
    t.transform.rotation.w      = quaternion[3]

    return t


class StaticEnvironment(object):
    def __init__(self):
        # Publishers
        self._world_robot_pub    = rospy.Publisher('/vicon/robot/robot/',      TransformStamped, queue_size=10)
        self._world_hololens_pub = rospy.Publisher('/vicon/hololens/hololens', TransformStamped, queue_size=10)
        self._hololens_pub       = rospy.Publisher('/hololens_position',       TransformStamped, queue_size=10)

        # Timers
        self._pub_timer = rospy.Timer(rospy.Duration.from_sec(0.1), self._publishers_cb)

        # Static Transform Msgs
        self.world_robot                            = TransformStamped()
        self.world_robot.header.frame_id            = "world"
        self.world_robot.child_frame_id             = "robot"
        self.world_robot.transform.translation.z    = 1.0
        self.world_robot.transform.rotation.z       = -0.7071068
        self.world_robot.transform.rotation.w       = 0.7071068

        self.world_hololens                         = TransformStamped()
        self.world_hololens.header.frame_id         = "world"
        self.world_hololens.child_frame_id          = "hololens"
        self.world_hololens.transform.translation.y = -1.0
        self.world_hololens.transform.translation.z = 1.0
        self.world_hololens.transform.rotation.z    = 0.7071068
        self.world_hololens.transform.rotation.w    = 0.7071068

        self.hololens                               = TransformStamped()
        self.hololens.header.frame_id               = "hololens"
        self.hololens.child_frame_id                = "holospace"
        self.hololens.transform.translation.y       = 1.0
        self.hololens.transform.rotation.z          = 0.7071068
        self.hololens.transform.rotation.w          = 0.7071068
        self.hololens = invert_transform(self.hololens)
        print(self.hololens)

    def _publishers_cb(self, event):
        """
        :type event: TimerEvent
        :param event:
        :return:
        """
        self.world_robot.header.stamp = event.current_real
        self.world_hololens.header.stamp = event.current_real
        self.hololens.header.stamp = event.current_real
        self._world_robot_pub.publish(self.world_robot)
        self._world_hololens_pub.publish(self.world_hololens)
        self._hololens_pub.publish(self.hololens)


if __name__ == "__main__":
    rospy.init_node("tf_static_environment", anonymous=True)
    env = StaticEnvironment()
    rospy.spin()
