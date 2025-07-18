#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped


class ParticleFilter:
    def __init__(self) -> None:
        self.point = None
        self.init = False


def callback_sub_point(msg, particle_filter):
    rospy.loginfo(f"[ParticleFilter]: Received new point")
    particle_filter.point = msg
    particle_filter.init = True


def callback_timer_viz(event, particle_filter, pub):
    if not particle_filter.init:
        return
    rospy.loginfo(f"[ParticleFilter]: Published new point")
    pub.publish(particle_filter.point)


if __name__ == "__main__":
    # Initialize the node: 'listener' is the name of the node
    rospy.init_node("particle_filter", anonymous=True)
    particle_filter = ParticleFilter()
    sub = rospy.Subscriber(
        "/uav1/rbl_controller/destination_point",
        PointStamped,
        lambda msg: callback_sub_point(msg, particle_filter),
    )
    pub = rospy.Publisher("/viz/filtered_point", PointStamped, queue_size=1)
    timer = rospy.Timer(
        rospy.Duration(0.1),
        lambda event: callback_timer_viz(event, particle_filter, pub),
    )
    # Keeps Python from exiting until this node is stopped
    rospy.spin()
