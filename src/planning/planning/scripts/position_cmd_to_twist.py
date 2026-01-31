#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from quadrotor_msgs.msg import PositionCommand


class PositionCmdToTwist(object):
    def __init__(self):
        self._frame_id = rospy.get_param("~frame_id", "")
        self._publish_stamped = rospy.get_param("~publish_stamped", True)
        self._publish_unstamped = rospy.get_param("~publish_unstamped", False)

        in_topic = rospy.get_param("~position_cmd_topic", "position_cmd")
        out_topic = rospy.get_param("~cmd_vel_topic", "cmd_vel")
        out_unstamped_topic = rospy.get_param("~cmd_vel_unstamped_topic", "cmd_vel_unstamped")

        self._pub = rospy.Publisher(out_topic, TwistStamped, queue_size=50) if self._publish_stamped else None
        self._pub_u = rospy.Publisher(out_unstamped_topic, Twist, queue_size=50) if self._publish_unstamped else None

        self._sub = rospy.Subscriber(in_topic, PositionCommand, self._cb, queue_size=50, tcp_nodelay=True)

        rospy.loginfo("[position_cmd_to_twist] in=%s out=%s (unstamped=%s)",
                      in_topic, out_topic, out_unstamped_topic)

    def _cb(self, msg):
        # linear velocity from planner, yaw rate -> angular.z
        if self._pub is not None:
            tws = TwistStamped()
            tws.header.stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
            tws.header.frame_id = self._frame_id if self._frame_id else (msg.header.frame_id or "world")
            tws.twist.linear.x = msg.velocity.x
            tws.twist.linear.y = msg.velocity.y
            tws.twist.linear.z = msg.velocity.z
            tws.twist.angular.z = msg.yaw_dot
            self._pub.publish(tws)

        if self._pub_u is not None:
            tw = Twist()
            tw.linear.x = msg.velocity.x
            tw.linear.y = msg.velocity.y
            tw.linear.z = msg.velocity.z
            tw.angular.z = msg.yaw_dot
            self._pub_u.publish(tw)


def main():
    rospy.init_node("position_cmd_to_twist")
    PositionCmdToTwist()
    rospy.spin()


if __name__ == "__main__":
    main()

