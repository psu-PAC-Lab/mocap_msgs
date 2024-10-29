import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from std_srvs.srv import Trigger
from std_srvs.srv import SetBool

import hiwonder

import numpy as np
import time

# tf2 is no longer supported in dashing https://github.com/ros2/geometry2/issues/188


class JetMaxOptitrackControlNode(Node):
    def __init__(self):
        super().__init__("jetmax_feedback_node")
        self.get_logger().info("Initializing jetmax_feedback_node")

        # initialize params
        self.target_position_tolerance = 0.01  # meters
        # target position of end_effector relative to base_link
        self.target_end_effector_t_base_link = None
        self.end_effector_t_base_link = None

        # initialize pose listener 
        self.pose_sub = self.create_subscription(
            PoseStamped, "base_link_pose", self.PoseSubCB, 10
        )
        # servo control listener, get target position, sw enable
        self.servo_control_sub = self.create_subscription(
            Bool, 'stop_servos', self.StopServosCB, 10
        )
        self.get_target_pos_sub = self.create_subscription(
            Bool, 'get_target_pos', self.GetTargetPosCB, 10
        )
        self.sw_robot_enable_sub = self.create_subscription(
            Bool, 'sw_robot_enable', self.SWRobotEnableCB, 10
        )

        self.robot_enabled = True

        # initialize arm and home
        self.jetmax = hiwonder.JetMax()
        self.jetmax.go_home(2)  # TODO: may need to add mutex to access servos
        time.sleep(2)
        self.StopServos()

        self.print_counter = 0
        self.print_coutner_top = 100

    def PoseSubCB(self, msg):
        self.end_effector_t_base_link = msg.pose.position

        if self.end_effector_t_base_link is not None:
            self.ActuateArm()

    def ActuateArm(self):
        self.print_counter += 1
        is_print = False
        if self.print_counter > self.print_coutner_top:
            is_print = True
            self.print_counter = 0
        
        if self.target_end_effector_t_base_link is None:
            if is_print: self.get_logger().info("target position not set")
            return

        d_x = self.target_end_effector_t_base_link.x - self.end_effector_t_base_link.x
        d_y = self.target_end_effector_t_base_link.y - self.end_effector_t_base_link.y
        d_z = self.target_end_effector_t_base_link.z - self.end_effector_t_base_link.z

        if is_print: self.get_logger().info(f"\nd_x: {d_x}\nd_y: {d_y}\nd_z: {d_z}")
        transform_translation = np.array([d_x, d_y, d_z])

        l2_norm = np.linalg.norm(transform_translation)
        if is_print: self.get_logger().info(f"l2_norm: {l2_norm}")
        if l2_norm < self.target_position_tolerance:
            if is_print: self.get_logger().info("robot within tolerance\n")
            return

        if is_print and self.robot_enabled: self.get_logger().info("moving robot\n")
        if is_print and not self.robot_enabled: self.get_logger().info("not moving robot\n")
        move_time = 0.25
        try:
            if self.robot_enabled: self.jetmax.set_position_relatively((-d_x*90, d_z*90, d_y*90), move_time)
        except Exception as ex:
            self.get_logger().info(f"Exception has occured: {ex}")
        time.sleep(move_time)

    def StopServosCB(self, msg):
        if msg.data:
            self.get_logger().info("Stopping servos")
            self.StopServos()
        else:
            self.get_logger().info("Servo control message received, but not stopping servos.")
    
    def GetTargetPosCB(self, msg):
        if self.end_effector_t_base_link is None:
            self.get_logger().info("Transform was not found")
            return
        self.target_end_effector_t_base_link = self.end_effector_t_base_link
        x = self.target_end_effector_t_base_link.x
        y = self.target_end_effector_t_base_link.y
        z = self.target_end_effector_t_base_link.z
        self.get_logger().info(f"Target Position: \nx: {x}\ny: {y}\nz: {z}")

    def SWRobotEnableCB(self, msg):
        if msg.data:
            self.robot_enabled = True
            self.get_logger().info("robot enabled")
            return
        else:
            self.robot_enabled = False
            self.get_logger().info("robot disabled")
            return

    def StopServos(self):
        # stops servos from making servo whine
        hiwonder.serial_servo_io.serial_serro_wirte_cmd(
            1, hiwonder.serial_servo_io.LOBOT_SERVO_MOVE_STOP
        )
        hiwonder.serial_servo_io.serial_serro_wirte_cmd(
            2, hiwonder.serial_servo_io.LOBOT_SERVO_MOVE_STOP
        )
        hiwonder.serial_servo_io.serial_serro_wirte_cmd(
            3, hiwonder.serial_servo_io.LOBOT_SERVO_MOVE_STOP
        )
        time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    node = JetMaxOptitrackControlNode()
    rclpy.spin(node)
    # node.StopServos()
    node.destroy_node()
    rclpy.shutdown()


# if __name__ == "__main__":
#     main()
