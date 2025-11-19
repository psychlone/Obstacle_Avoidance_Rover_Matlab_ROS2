#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import random
import math

class RandomGoalSender(Node):
    def __init__(self):
        super().__init__('random_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.map_range = 3.0  # area (meters) within which to randomize goals
        self.timer = self.create_timer(10.0, self.send_random_goal)

    def send_random_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('NavigateToPose action not available yet.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Generate random (x, y) within map_range
        goal_msg.pose.pose.position.x = random.uniform(-self.map_range, self.map_range)
        goal_msg.pose.pose.position.y = random.uniform(-self.map_range, self.map_range)
        goal_yaw = random.uniform(-math.pi, math.pi)

        # convert yaw to quaternion
        qz = math.sin(goal_yaw / 2.0)
        qw = math.cos(goal_yaw / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f'Sending random goal: ({goal_msg.pose.pose.position.x:.2f}, '
                               f'{goal_msg.pose.pose.position.y:.2f})')
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RandomGoalSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
