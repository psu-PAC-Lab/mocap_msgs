import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

import tf2_ros

class SteeringDemoNode(Node):
    def __init__(self):
        super().__init__("steering_demo")
        self.get_logger().info("Starting Steering demo node")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1/10, self.timer_callback)
        self.transform = None
        self.reference_position = None

        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_reference_position_sub = self.create_subscription(
            Bool, "get_reference_position", self.get_reference_position_callback, 10
        )
    
    def timer_callback(self):
        try:
            self.transform = self.tf_buffer.lookup_transform('map', 'steering_wheel', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warning("Transform lookup failed")
        if self.reference_position is None:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            return
        cmd_vel_msg = Twist()
        translation = self.transform.transform.translation
        cmd_vel_msg.linear.x = translation.z - self.reference_position.z * 0.25
        cmd_vel_msg.angular.z = translation.y - self.reference_position.y * 4.0
        self.cmd_vel_publisher.publish(cmd_vel_msg)
    
    def get_reference_position_callback(self, msg):
        if not msg.data:
            self.reference_position = None
        if self.transform is None:
            self.get_logger().info("Transform was not found")
            return
        self.reference_position = self.transform.transform.translation
        x = self.reference_position.x
        y = self.reference_position.y
        z = self.reference_position.z
        self.get_logger().info(f"Reference Position: \nx: {x}\ny: {y}\nz: {z}")
            

def main(args=None):
    rclpy.init(args=args)
    node = SteeringDemoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
