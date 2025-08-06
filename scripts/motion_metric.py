#!/usr/bin/env python

import array

from matplotlib import axis
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from mrs_msgs.msg import Float64Stamped
from geometry_msgs.msg import PoseStamped


class UavState:
    def __init__(self, uav_id, history_size, odom_topic) -> None:
        self.uav_id = uav_id
        self.history_size = history_size
        self.sub_odom = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.velocity = []
        self.unit_velocity = []
        self.position = None

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

        self.position = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]
        )


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
        self.pub_closest_point = rospy.Publisher(
            "/motion_metric/closest", PoseStamped, queue_size=10
        )
        self.pub_farthest_point = rospy.Publisher(
            "/motion_metric/farthest", PoseStamped, queue_size=10
        )
        self.pub_desired_target = rospy.Publisher(
            "/motion_metric/desired_target", Odometry, queue_size=10
        )
        self.goal = np.array([0.0, 60.0, 2.0])

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

            if all(uav.position is not None for uav in self.uav_states):
                possible_targets = [uav.position for uav in self.uav_states[1:]]
                value = self.get_coop_importance_value(
                    self.goal, self.uav_states[0].position, possible_targets
                )
                print(f"value observed by UAV1: {value}")

                desired_target = possible_targets[np.argmax(value)]
                msg_target = Odometry()
                msg_target.header.frame_id = "common_origin"
                msg_target.header.stamp = rospy.Time(0)
                msg_target.pose.pose.position.x = desired_target[0]
                msg_target.pose.pose.position.y = desired_target[1]
                msg_target.pose.pose.position.z = desired_target[2]

                self.pub_desired_target.publish(msg_target)

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

    def get_closest_position(self, ref_position, other_positions):
        rel_positions = np.stack(other_positions, axis=0) - np.tile(
            ref_position, (len(other_positions), 1)
        )
        return other_positions[np.argmin(np.linalg.norm(rel_positions, axis=1))]

    def get_farthest_position(self, ref_position, other_positions):
        rel_positions = np.stack(other_positions, axis=0) - np.tile(
            ref_position, (len(other_positions), 1)
        )
        return other_positions[np.argmax(np.linalg.norm(rel_positions, axis=1))]

    def get_coop_importance_value(
        self, goal_position, uav_position, neighbor_positions, alpha=0.5
    ):
        position_wrt_uav = np.stack(neighbor_positions, axis=0) - np.tile(
            uav_position, (len(neighbor_positions), 1)
        )
        dist_to_uav = np.linalg.norm(position_wrt_uav, axis=1)
        position_wrt_goal = np.stack(neighbor_positions, axis=0) - np.tile(
            goal_position, (len(neighbor_positions), 1)
        )
        dist_to_goal = np.linalg.norm(position_wrt_goal, axis=1)

        value = alpha * np.log(1.0 / dist_to_goal) + (1 - alpha) * np.log(dist_to_uav)
        return value


if __name__ == "__main__":
    try:
        node = MotionMetric(uav_count=4)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
