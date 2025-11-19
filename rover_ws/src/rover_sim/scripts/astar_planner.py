#!/usr/bin/env python3
import rclpy, math, heapq, numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

# ---------- A* with 8-directional moves ----------
def astar(grid, start, goal):
    rows, cols = grid.shape

    def H(a,b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    moves = [
        (1,0),(-1,0),(0,1),(0,-1),
        (1,1),(1,-1),(-1,1),(-1,-1)
    ]

    openq = [(0, start)]
    came_from, g = {}, {start:0}
    f = {start:H(start,goal)}

    while openq:
        _, cur = heapq.heappop(openq)
        if cur == goal:
            path=[cur]
            while cur in came_from:
                cur=came_from[cur]
                path.append(cur)
            return path[::-1]
        for mv in moves:
            n=(cur[0]+mv[0],cur[1]+mv[1])
            if 0<=n[0]<rows and 0<=n[1]<cols and grid[n]==0:
                cost = math.hypot(mv[0], mv[1])
                ng=g[cur]+cost
                if n not in g or ng<g[n]:
                    came_from[n]=cur
                    g[n]=ng
                    f[n]=ng+H(n,goal)
                    heapq.heappush(openq,(f[n],n))
    return []
# -----------------------------------------------

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Grid setup: 10×10 m → 100×100 cells @ 0.1 m
        self.res = 0.1
        self.size = int(10 / self.res)
        self.grid = np.zeros((self.size, self.size), dtype=int)

        # Obstacles matching obstacle_world.world
        self.rect_obstacle(x=1.0, y=0.0, sx=0.5, sy=0.5)
        self.rect_obstacle(x=2.0, y=1.0, sx=0.8, sy=0.3)

        # Inflate obstacles by rover radius (~0.35 m)
        self.inflate_obstacles(0.35)

        self.pose = None
        self.path = []
        self.goal_world = None
        self.wp_tol = 0.2

    # ---------- Grid helpers ----------
    def w2g(self, x, y):
        return int((x + 5.0)/self.res), int((y + 5.0)/self.res)
    def g2w(self, gx, gy):
        return gx*self.res - 5.0, gy*self.res - 5.0

    # ---------- Obstacle utilities ----------
    def rect_obstacle(self, x, y, sx, sy):
        def to_idx(wx, wy):
            gx = int((wx + 5.0)/self.res); gy = int((wy + 5.0)/self.res)
            return gx, gy
        gx0, gy0 = to_idx(x - sx/2, y - sy/2)
        gx1, gy1 = to_idx(x + sx/2, y + sy/2)
        gx0, gx1 = max(0,gx0), min(self.size-1,gx1)
        gy0, gy1 = max(0,gy0), min(self.size-1,gy1)
        self.grid[gx0:gx1+1, gy0:gy1+1] = 1

    def inflate_obstacles(self, inflation_radius=0.35):
        cells = int(inflation_radius / self.res)
        new_grid = self.grid.copy()
        for i in range(self.size):
            for j in range(self.size):
                if self.grid[i,j]==1:
                    xmin, xmax = max(0,i-cells), min(self.size-1,i+cells)
                    ymin, ymax = max(0,j-cells), min(self.size-1,j+cells)
                    new_grid[xmin:xmax+1, ymin:ymax+1] = 1
        self.grid = new_grid

    # ---------- Callbacks ----------
    def odom_cb(self, msg):
        self.pose = msg.pose.pose

    def goal_cb(self, msg):
        self.goal_world = (msg.pose.position.x, msg.pose.position.y)
        self.plan_new_path()

    # ---------- Planner ----------
    def plan_new_path(self):
        if self.pose is None or self.goal_world is None:
            return
        s = self.w2g(self.pose.position.x, self.pose.position.y)
        g = self.w2g(*self.goal_world)

        # Ensure start/goal not in obstacles
        s = self._nearest_free(s)
        g = self._nearest_free(g)

        grid_path = astar(self.grid, s, g)
        self.path = [self.g2w(p[0], p[1]) for p in grid_path]

        if not self.path:
            self.get_logger().warn('A*: no path found. Stopping rover.')
            self.pub_cmd.publish(Twist())
            return
        self.get_logger().info(f'Planned path with {len(self.path)} waypoints.')

    def _nearest_free(self, cell):
        """If cell inside obstacle, move to nearest free cell."""
        if self.grid[cell] == 0:
            return cell
        for r in range(1,10):
            for dx in range(-r,r+1):
                for dy in range(-r,r+1):
                    nx, ny = cell[0]+dx, cell[1]+dy
                    if 0<=nx<self.size and 0<=ny<self.size and self.grid[nx,ny]==0:
                        return (nx,ny)
        return cell

    # ---------- Control loop ----------
    def control_loop(self):
        if self.pose is None:
            return

        # stop if no path
        if not self.path:
            self.pub_cmd.publish(Twist())
            return

        tx, ty = self.path[0]
        dx = tx - self.pose.position.x
        dy = ty - self.pose.position.y
        dist = math.hypot(dx, dy)
        yaw = self.yaw(self.pose.orientation)
        head = math.atan2(dy, dx)
        ang = self.wrap(head - yaw)

        cmd = Twist()
        if dist < self.wp_tol:
            self.path.pop(0)
            if not self.path:
                self.get_logger().info('Goal reached. Stopping.')
                self.pub_cmd.publish(Twist())
            return

        cmd.linear.x = max(0.1, min(0.35, 0.6*dist))
        cmd.angular.z = max(-1.2, min(1.2, 2.0*ang))
        self.pub_cmd.publish(cmd)

    # ---------- Utility ----------
    @staticmethod
    def yaw(q):
        s = 2.0*(q.w*q.z + q.x*q.y)
        c = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        return math.atan2(s, c)
    @staticmethod
    def wrap(a):
        while a>math.pi: a -= 2*math.pi
        while a<-math.pi: a += 2*math.pi
        return a

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # stop rover before exit
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
