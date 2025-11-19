#!/usr/bin/env python3
import rclpy, math, random
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Twist
from gazebo_msgs.srv import SpawnEntity, SetEntityState
from gazebo_msgs.msg import EntityState

class RandomGoalPublisher(Node):
    def __init__(self):
        super().__init__('random_goal_publisher')
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(8.0, self.publish_goal)

        # parameters
        self.local_radius_min = 0.5   # choose a nearby goal
        self.local_radius_max = 1.5
        self.world_limit = 4.5        # clamp goals to [-4.5, 4.5] in odom

        # gazebo goal marker
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.set_state_cli = self.create_client(SetEntityState, '/set_entity_state')
        self.marker_spawned = False
        self.marker_name = 'goal_marker'

    def publish_goal(self):
        # pick a random pose in a ring around current origin of odom
        radius = random.uniform(self.local_radius_min, self.local_radius_max)
        theta  = random.uniform(-math.pi, math.pi)
        x = max(min(radius*math.cos(theta),  self.world_limit), -self.world_limit)
        y = max(min(radius*math.sin(theta),  self.world_limit), -self.world_limit)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0  # yaw = 0

        self.pub.publish(msg)
        self.get_logger().info(f'New goal near robot: ({x:.2f}, {y:.2f})')

        # show / move marker in Gazebo
        self._show_marker_in_gazebo(x, y)

    def _show_marker_in_gazebo(self, x, y):
        if not self.marker_spawned:
            # wait for /spawn_entity
            if not self.spawn_cli.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn('spawn_entity not ready yet')
                return

            # small green cylinder
            sdf = f"""
<sdf version='1.6'>
  <model name='{self.marker_name}'>
    <static>true</static>
    <pose>{x} {y} 0.05 0 0 0</pose>
    <link name='link'>
      <visual name='vis'>
        <geometry><cylinder><radius>0.07</radius><length>0.10</length></cylinder></geometry>
        <material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material>
      </visual>
      <collision name='col'>
        <geometry><cylinder><radius>0.07</radius><length>0.10</length></cylinder></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""
            req = SpawnEntity.Request()
            req.name = self.marker_name
            req.xml = sdf
            req.robot_namespace = ''
            req.reference_frame = 'world'  # Gazebo world

            fut = self.spawn_cli.call_async(req)
            rclpy.task.Future.add_done_callback(fut, lambda _: None)
            self.marker_spawned = True
        else:
            # move marker using /set_entity_state
            if not self.set_state_cli.wait_for_service(timeout_sec=0.2):
                return
            st = EntityState()
            st.name = self.marker_name
            st.pose.position.x = x
            st.pose.position.y = y
            st.pose.position.z = 0.05
            st.pose.orientation.w = 1.0
            req2 = SetEntityState.Request()
            req2.state = st
            self.set_state_cli.call_async(req2)

def main(args=None):
    rclpy.init(args=args)
    node = RandomGoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
