#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from exercise_interfaces.msg import QuadrotorGoal
import numpy as np


class QuadrotorGoalPublisher(Node):
    def __init__(self):
        super().__init__('quadrotor_goal_publisher')
        queue_size = 10
        self.goal_publisher = self.create_publisher(QuadrotorGoal, '/quadrotor_goal', queue_size)
        self.goalRate = self.create_timer(0.5, self.get_goal)

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_z = 0.0
        self.goal_angle = 0.0

    def get_goal(self):
        self.get_logger().info('GetGoal')
        goal = QuadrotorGoal()
        goal.x = float(input("Set your x goal:"))
        goal.y = float(input("Set your y goal:"))
        goal.z = float(input("Set your z goal:"))
        angle_deg = float(input("Set your angle goal in degrees:"))
        goal.angle = angle_deg*(np.pi/180)
        self.publish_goal(goal)
        print("goal selected")

    def publish_goal(self, goal):
        self.goal_publisher.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    quadrotor_goal_publischer = QuadrotorGoalPublisher()
    try:
        rclpy.spin(quadrotor_goal_publischer)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        quadrotor_goal_publischer.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
