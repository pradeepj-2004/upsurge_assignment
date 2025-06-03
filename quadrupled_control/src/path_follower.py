#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

import math


class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_node')

        self.sub_cb_group = ReentrantCallbackGroup()
        self.timer_cb_group = ReentrantCallbackGroup()

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry,'/odom/ground_truth',self.odom_callback,10,callback_group=self.sub_cb_group)
        self.timer = self.create_timer(0.5, self.traj_callback, callback_group=self.timer_cb_group)
        self.rate = self.create_rate(10) 

        self.working=False
        self.initialise = False
        self.state=0
        self.position_x = 0.0
        self.position_y = 0.0
        self.ang_pos = 0.0




        self.get_logger().info("TrajectoryFollower node has been started.")

    def publish_cmd_vel(self, x, z):
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = z
        self.publisher_.publish(msg)

    def odom_callback(self, msg: Odometry):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y  
        orientation_q = msg.pose.pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        self.ang_pos = yaw

        if not self.initialise:
            self.initialise = True
            self.get_logger().info("Odometry initialized.")

    def moveforward(self, target_distance):
        start_x = self.position_x
        start_y = self.position_y

        self.get_logger().info(f"Moving forward for {target_distance} meters...")

        while rclpy.ok():
            dx = self.position_x - start_x
            dy = self.position_y - start_y
            current_distance = math.sqrt(dx**2 + dy**2)

            if current_distance >= target_distance:
                break

            self.publish_cmd_vel(0.1, 0.0)
            self.rate.sleep()
        self.publish_cmd_vel(0.0, 0.0)
        self.get_logger().info("Target distance reached.")
        self.working=False

    def turn_robot(self, target_angle):

        self.get_logger().info(f"Turning robot to {target_angle} rad...")

        while rclpy.ok():
            dtheta = self.ang_pos - target_angle
            if abs(dtheta) <= 0.05:
                break

            self.publish_cmd_vel(0.0, 0.04)
            self.rate.sleep()

        self.publish_cmd_vel(0.0, 0.0)
        self.get_logger().info("Target angle reached.")
        self.working=False

    def traj_callback(self):
        if self.initialise and self.state==0:
            if self.working==False:
                self.working=True
                self.moveforward(2)
            self.state=1
        elif self.initialise and self.state==1:
            if self.working==False:
                self.working=True
                self.turn_robot(1.57)
            self.state=2

        elif self.initialise and self.state==2:
            if self.working==False:
                self.working=True
                self.moveforward(2)
            self.state=3
        elif self.initialise and self.state==3:
            if self.working==False:
                self.working=True
                self.turn_robot(3.14)
            self.state=4
        elif self.initialise and self.state==4:
            if self.working==False:
                self.working=True
                self.moveforward(2)
            self.state=5
        
        elif self.initialise and self.state==5:
            if self.working==False:
                self.working=True
                self.turn_robot(-1.57)
            self.state=6

        elif self.initialise and self.state==6:
            if self.working==False:
                self.working=True
                self.moveforward(2)
            self.state=7

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollower()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
