#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_ros

class TransformRepublisher(Node):
    def __init__(self):
        super().__init__('transform_republisher')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pose_publisher = self.create_publisher(PoseStamped, 'base_link_pose', 10)
        self.timer = self.create_timer(1/30, self.timer_callback)

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform('base_link', 'end_effector', rclpy.time.Time())
            pose_msg = PoseStamped()
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            pose_msg.pose.orientation = transform.transform.rotation
            self.pose_publisher.publish(pose_msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warning("Transform lookup failed")

def main(args=None):
    rclpy.init(args=args)
    transform_republisher = TransformRepublisher()
    rclpy.spin(transform_republisher)
    transform_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
