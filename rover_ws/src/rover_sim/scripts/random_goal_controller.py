#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import random, math

class RandomGoalController(Node):
    def __init__(self):
        super().__init__('random_goal_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.pose = None
        self.goal = None
        self.laser_ranges = []
        self.safe_distance = 0.5
        self.map_range = 3.0

    def odom_cb(self, msg):
        self.pose = msg.pose.pose

    def scan_cb(self, msg):
        self.laser_ranges = msg.ranges

    def random_goal(self):
        """Pick a random goal near the current position."""
        if self.pose is None:
            # fallback to origin-centered random if no pose yet
            x = random.uniform(-self.map_range, self.map_range)
            y = random.uniform(-self.map_range, self.map_range)
        else:
            # current position
            cx = self.pose.position.x
            cy = self.pose.position.y

            # random small step: radius 0.5–1.5 m in random direction
            step = random.uniform(0.5, 1.5)
            theta = random.uniform(-math.pi, math.pi)
            x = cx + step * math.cos(theta)
            y = cy + step * math.sin(theta)

        self.get_logger().info(f'New nearby goal: ({x:.2f}, {y:.2f})')
        return (x, y)


    def control_loop(self):
        if self.pose is None:
            return

        if self.goal is None:
            self.goal = self.random_goal()
            return

        x = self.pose.position.x
        y = self.pose.position.y
        gx, gy = self.goal

        dx, dy = gx - x, gy - y
        distance = math.hypot(dx, dy)
        yaw = self.yaw_from_quaternion(self.pose.orientation)
        heading = math.atan2(dy, dx)
        angle_err = self.angle_wrap(heading - yaw)

        cmd = Twist()

        # Simple obstacle check using front 30°
        obstacle_ahead = False
        if self.laser_ranges:
            mid = len(self.laser_ranges)//2
            window = int(15)
            front = self.laser_ranges[mid-window:mid+window]
            if any(r < self.safe_distance for r in front if r > 0.0):
                obstacle_ahead = True

        if obstacle_ahead:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # rotate to clear obstacle
            self.get_logger().info("Obstacle detected - turning")
        elif distance > 0.2:
            # Move toward goal
            cmd.linear.x = 0.3
            cmd.angular.z = 1.5 * angle_err
        else:
            self.get_logger().info("Goal reached!")
            self.goal = None  # pick new goal next cycle

        self.cmd_pub.publish(cmd)

    @staticmethod
    def yaw_from_quaternion(q):
        siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def angle_wrap(a):
        while a > math.pi:  a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

def main(args=None):
    rclpy.init(args=args)
    node = RandomGoalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
