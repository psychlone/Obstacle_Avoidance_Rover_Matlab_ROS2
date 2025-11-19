#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class PotentialFieldPlanner(Node):
    def __init__(self):
        super().__init__('potential_field_planner')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # parameters
        self.pose = None
        self.goal = None
        self.scan = []
        self.goal_tol = 0.3
        self.k_att = 1.0      # attraction gain
        self.k_rep = 0.8      # repulsion gain
        self.rep_range = 1.0  # obstacle influence distance (m)

    def odom_cb(self, msg): self.pose = msg.pose.pose
    def scan_cb(self, msg): self.scan = msg.ranges
    def goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"New goal received: {self.goal}")

    def control_loop(self):
        if self.pose is None or self.goal is None:
            return

        # position and heading
        px, py = self.pose.position.x, self.pose.position.y
        yaw = self.get_yaw(self.pose.orientation)

        # attraction vector
        gx, gy = self.goal
        dx, dy = gx - px, gy - py
        dist_goal = math.hypot(dx, dy)
        if dist_goal < self.goal_tol:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("Goal reached!")
            self.goal = None
            return
        att_x = self.k_att * dx
        att_y = self.k_att * dy

        # repulsive vector (from LiDAR)
        rep_x, rep_y = 0.0, 0.0
        if self.scan:
            angle_min = -math.pi
            angle_inc = (2*math.pi)/len(self.scan)
            for i, r in enumerate(self.scan):
                if 0.05 < r < self.rep_range:
                    angle = angle_min + i*angle_inc + yaw
                    strength = self.k_rep * (1.0/r - 1.0/self.rep_range) / (r*r)
                    rep_x -= strength * math.cos(angle)
                    rep_y -= strength * math.sin(angle)

        # total vector
        fx = att_x + rep_x
        fy = att_y + rep_y
        heading = math.atan2(fy, fx)
        turn = self.angle_wrap(heading - yaw)

        cmd = Twist()
        cmd.angular.z = 1.5 * turn
        cmd.linear.x = max(0.0, min(0.4, 0.5 * dist_goal * math.cos(turn)))

        # stop if something directly ahead closer than 0.3 m
        if self.scan and min(self.scan) < 0.3:
            cmd.linear.x = 0.0
            self.get_logger().debug("Stopping, obstacle too close.")

        self.cmd_pub.publish(cmd)

    @staticmethod
    def get_yaw(q):
        s = 2.0*(q.w*q.z + q.x*q.y)
        c = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        return math.atan2(s, c)

    @staticmethod
    def angle_wrap(a):
        while a > math.pi:  a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
