#!/usr/bin/env python

import array

from matplotlib import axis
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class OrderMetric:
    def __init__(self):
        rospy.init_node("order_metric")

        # Store the linear velocity vectors from each odometry message
        self.linear_vels = [None] * 4

        # Subscribers for each odometry input
        self.subs = [
            rospy.Subscriber(
                "/uav1/estimation_manager/odom_main",
                Odometry,
                self.odom_callback,
                callback_args=0,
            ),
            rospy.Subscriber(
                "/uav2/estimation_manager/odom_main",
                Odometry,
                self.odom_callback,
                callback_args=1,
            ),
            rospy.Subscriber(
                "/uav3/estimation_manager/odom_main",
                Odometry,
                self.odom_callback,
                callback_args=2,
            ),
            rospy.Subscriber(
                "/uav4/estimation_manager/odom_main",
                Odometry,
                self.odom_callback,
                callback_args=3,
            ),
        ]

        # Publisher for the calculated dot product
        self.pub = rospy.Publisher("/order_metric", Float64, queue_size=10)
        rospy.loginfo("Initialized")

    def odom_callback(self, msg, index):
        # Extract the linear velocity vector
        v = msg.twist.twist.linear
        self.linear_vels[index] = np.array([v.x, v.y, v.z]) / np.linalg.norm(
            [v.x, v.y, v.z]
        )

        # Proceed only when all four messages are received at least once
        if all(vec is not None for vec in self.linear_vels):
            order = self.get_order_metric(self.linear_vels)
            self.pub.publish(order)
            rospy.loginfo(f"Publishing order_metric = {order}")

    def get_order_metric(self, velocities):
        prod = np.stack(velocities, axis=0) @ np.stack(velocities, axis=1)
        order = (
            1.0
            / (len(velocities) * (len(velocities) - 1))
            * np.sum(prod - np.diag(np.diag(prod)))
        )
        return order


if __name__ == "__main__":
    try:
        node = OrderMetric()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
