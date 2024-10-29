import rclpy
from rclpy.node import Node
from hiwonder_acker_driver.ackermann import AckermannChassis
from geometry_msgs.msg import Twist

class AckerController(Node):
    def __init__(self):
        super().__init__("acker_controller")
        self.get_logger().info("Starting the Hiwonder Ackermann driver node")

        self.last_linear_x = 0
        self.last_linear_y = 0
        self.last_angular_z = 0

        self.ackermann = AckermannChassis(
            track=0.222, 
            wheelbase=0.213, 
            wheel_diameter=101, 
            pulse_per_cycle=4*11*90
        )  #44.0 * 90.0
        
        self.go_factor = 1.0
        self.turn_factor = 1.0
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )
    
    def app_cmd_vel_callback(self, msg):
        if msg.linear.x > 0.2:
            msg.linear.x = 0.2
        if msg.linear.x < -0.2:
            msg.linear.x = -0.2
        if msg.linear.y > 0.2:
            msg.linear.y = 0.2
        if msg.linear.y < -0.2:
            msg.linear.y = -0.2
        if msg.angular.z > 0.5:
            msg.angular.z = 0.5
        if msg.angular.z < -0.5:
            msg.angular.z = -0.5
        self.cmd_vel_callback(msg)

    def cmd_vel_callback(self, msg):
        if abs(msg.linear.y) > 1e-8:
            linear_x = 0
        else:
            linear_x = self.go_factor * msg.linear.x
        linear_y = 0
        if msg.angular.z != 0:
            r = msg.linear.x / msg.angular.z
            if r != 0:
                angular_z = linear_x / r
            else:
                angular_z = 0
        else:
            angular_z = 0
        
        self.last_linear_x = linear_x
        self.last_linear_y = linear_y
        self.last_angular_z = angular_z
        if abs(msg.linear.z) > 1e-8 and abs(msg.linear.x) < 1e-8 and abs(msg.angular.z) < 1e-8:
            self.ackermann.set_velocity(linear_x, angular_z, reset_servo=False)
        else:
            self.ackermann.set_velocity(linear_x, angular_z)


def main(args=None):
    rclpy.init(args=args)
    node = AckerController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
