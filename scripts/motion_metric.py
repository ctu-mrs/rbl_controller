#!/usr/bin/env python

import array

from matplotlib import axis
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from mrs_msgs.msg import Float64Stamped


class UavState:
    def __init__(self, uav_id, history_size, odom_topic) -> None:
        self.uav_id = uav_id
        self.history_size = history_size
        self.sub_odom = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.velocity = []
        self.unit_velocity = []

    def odom_callback(self, msg):
        # Extract the linear velocity vector
        v = msg.twist.twist.linear
        if len(self.velocity) >= self.history_size:
            self.velocity.pop(0)
            self.unit_velocity.pop(0)
        self.velocity.append(np.array([v.x, v.y, v.z]))
        self.unit_velocity.append(
            np.array([v.x, v.y, v.z]) / np.linalg.norm([v.x, v.y, v.z])
        )
        assert len(self.velocity) <= self.history_size
        assert len(self.unit_velocity) <= self.history_size


class MotionMetric:
    def __init__(self, *, uav_count=0):
        rospy.init_node("motion_metric")
        self.uav_states = []
        self.pub_path_pers = {}

        for i in range(uav_count):
            self.uav_states.append(
                UavState(
                    "uav" + str(i + 1),
                    100,
                    "/uav" + str(i + 1) + "/estimation_manager/odom_main",
                )
            )
            self.pub_path_pers["uav" + str(i + 1)] = rospy.Publisher(
                "/motion_metric/uav" + str(i + 1) + "/path_persistence",
                Float64Stamped,
                queue_size=10,
            )

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.pub_order = rospy.Publisher(
            "/motion_metric/order", Float64Stamped, queue_size=10
        )

        rospy.loginfo("Initialized")

    def timer_callback(self, e):
        if len(self.uav_states) > 0:
            if all(len(uav.unit_velocity) > 0 for uav in self.uav_states):
                unit_velocities = [uav.unit_velocity[-1] for uav in self.uav_states]
                msg_order = Float64Stamped()
                msg_order.header.stamp = rospy.Time.now()
                msg_order.value = self.get_order_metric(unit_velocities)
                self.pub_order.publish(msg_order)
                rospy.loginfo(f"Publishing order: {msg_order.value}")

                for uav in self.uav_states:
                    if len(uav.unit_velocity) > 1:
                        msg_path_pers = Float64Stamped()
                        msg_path_pers.header.stamp = rospy.Time.now()
                        msg_path_pers.value = self.get_path_pers_metric(
                            uav.unit_velocity
                        )
                        self.pub_path_pers[uav.uav_id].publish(msg_path_pers)
                        rospy.loginfo(
                            f"Publishing path persistence: {msg_path_pers.value}"
                        )

    def get_order_metric(self, unit_vecs):
        prod = np.stack(unit_vecs, axis=0) @ np.stack(unit_vecs, axis=1)
        order = np.sum(prod - np.diag(np.diag(prod))) / (
            len(unit_vecs) * (len(unit_vecs) - 1)
        )
        return order

    def get_path_pers_metric(self, unit_vecs):
        prod = np.stack(unit_vecs[:-1], axis=0) @ np.stack(unit_vecs[1:], axis=1)
        path_pers = np.sum(np.diag(prod)) / (len(unit_vecs) - 1)
        return path_pers


if __name__ == "__main__":
    try:
        node = MotionMetric(uav_count=4)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
