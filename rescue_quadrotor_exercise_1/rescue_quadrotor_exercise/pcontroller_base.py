#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from exercise_interfaces.msg import QuadrotorGoal
import transforms3d
from transforms3d.euler import euler2mat, mat2euler
from transforms3d._gohlketransforms import inverse_matrix
import numpy as np 

class controller(Node):
    def __init__(self):
        super().__init__('p_controller')
        queue_size = 10
        self.goal = 0
        self.goal_x = 0
        self.goal_y = 0
        self.goal_z = 0
        self.goal_angle = 0

        self.pose_subscriber = self.create_subscription(
            PoseStamped, '/ground_truth_to_tf/pose', self.update_pose, queue_size)
        self.goal_subscriber = self.create_subscription(
            QuadrotorGoal, '/quadrotor_goal', self.update_goal, queue_size)
        # define a publisher that publishes Twist messages on the /cmd_vel topic periodically
        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 100)
        self.goalRate = self.create_timer(0.001, self.move2goal)

        self.pose_stamped = PoseStamped()
        self.roll, self.pitch, self.yaw = self.quaternion_to_euler(self.pose_stamped.pose)

    def update_pose(self, pose_msg):
        # update current pose of the robot
        self.pose_stamped = pose_msg
        self.roll, self.pitch, self.yaw = self.quaternion_to_euler(pose_msg.pose)

    def update_goal(self, msg):
        # update goal with new received goal from QuadrotorGoalPublisher over /quadrotor_goal topic
        self.goal = msg
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.goal_z = msg.z
        self.goal_angle = msg.angle

    def quaternion_to_euler(self, robot_pose):
        # transforms robot position from quaternion to roll, pitch, yaw
        quaternion = (
            robot_pose.orientation.w,
            robot_pose.orientation.x,
            robot_pose.orientation.y,
            robot_pose.orientation.z)

        euler = transforms3d.euler.quat2euler(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        return roll, pitch, yaw

    def control_law(self):
        ux = 3*(self.goal_x - self.pose_stamped.pose.position.x)
        uy = 2*(self.goal_y - self.pose_stamped.pose.position.y)
        uz = 3*(self.goal_z - self.pose_stamped.pose.position.z)
        uw = 0.04*(self.goal_angle - self.yaw)
        rotmat = inverse_matrix(euler2mat(self.roll +self.yaw, self.pitch, 0, 'sxyz'))
        vel = np.dot(rotmat,np.array([ux, uy, uz]))
        return vel[0], vel[1], vel[2], uw

    def move2goal(self):
        ux, uy, uz, uw = self.control_law()
        twistcmd = Twist()
        twistcmd.linear.x = ux
        twistcmd.linear.y = uy
        twistcmd.linear.z = uz
        twistcmd.angular.x = 0.0
        twistcmd.angular.y = 0.0
        twistcmd.angular.z = uw

        # define a Twist message here, apply the values from the control lab and
        # publish it to cmd_vel
        self.twist_publisher.publish(twistcmd)


def main(args=None):
    rclpy.init(args=args)
    p_controller = controller()
    try:
        print("spinning p_controller")
        rclpy.spin(p_controller)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        p_controller.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
