"""Module designed to make the robot explore rooms."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
import numpy as np
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid


class AutomaticExplore(Node):
    """Ros2 Node designed to implement exploration algorithm."""

    def __init__(self):
        """Create AutomaticExplore Node."""
        super().__init__('explore_node')
        self.get_logger().info('Automatic Explore Node Started!')
        self.logger = self.get_logger()

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.VOLATILE)

        self.costmap_subscriber = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.update_costmap, qos)

        self.create_service(Empty, 'start_navigation', self.start_navigation)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_client.wait_for_server()
        self.get_logger().info('Navigator is ready.')

        self.costmap_data = None
        self.map_info = None

    def get_unexplored_cells(self):
        """Get unexplored Cells."""
        unexplored_cell = np.argwhere(self.costmap_data == -1)
        if unexplored_cell.size == 0:
            self.logger.info('No more unexplored areas.')
        return unexplored_cell

    def move_to_target(self, target):
        """Move robot to target position."""
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.pose.position.x = target['x']
        target_pose.pose.position.y = target['y']
        target_pose.pose.position.z = 0.0
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 1.0
        target_pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(f'Navigating to unexplored target at ({target_pose})')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        # Send goal to the action server
        future = self.nav_client.send_goal_async(goal_msg)
        self.get_logger().info("Waiting for goal to be accepted...")
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal was rejected by the action server.')
            return False

        self.get_logger().info('Goal accepted. Waiting for result...')

        # # Wait for the result
        # result_future = goal_handle.get_result_async()
        # rclpy.spin_until_future_complete(self, result_future)

        # result = result_future.result()
        # if result.status == 4:  # SUCCEEDED
        #     self.get_logger().info('Target goal reached')
        #     return True
        # else:
        #     self.get_logger().warning(f'Goal failed with status: {result.status}')
        #     return False

    def least_expensive_unexplored(self, unexplored_cell):
        """Get least expensive unexplored cell."""
        if unexplored_cell.size == 0:
            return None, unexplored_cell

        target = None
        robot_x, robot_y = 0, 0  # Replace with actual robot position
        min_cost = float('inf')
        removed_index = None

        for index, (cell_y, cell_x) in enumerate(unexplored_cell):
            target_x = self.map_info.origin.position.x + (cell_x + 0.5) * self.map_info.resolution
            target_y = self.map_info.origin.position.y + (cell_y + 0.5) * self.map_info.resolution

            distance = np.hypot(target_x - robot_x, target_y - robot_y)
            if distance < min_cost:
                min_cost = distance
                target = {'x': target_x, 'y': target_y}
                removed_index = index

        if target is None:
            return None, unexplored_cell

        unexplored_cell = np.delete(unexplored_cell, removed_index, axis=0)
        return target, unexplored_cell

    def start_navigation(self, request, response):
        """Start Nevigation."""
        if self.costmap_data is None or self.map_info is None:
            return None

        unexplored_cell = self.get_unexplored_cells()
        while unexplored_cell.size > 0:

            # get the least expensive goal
            target, unexplored_cell = self.least_expensive_unexplored(unexplored_cell)

            if target is None:
                self.get_logger().info('No valid unexplored target found.')
                break

            # move towards the target
            self.get_logger().info(f'Navigating to target: {target}')
            success = self.move_to_target(target)
            self.logger.info('done with navigation')

            if not success:
                self.get_logger().warning('Navigation to target failed. Skipping this target.')

            unexplored_cell = self.get_unexplored_cells()  # Refresh unexplored cells after moving

        self.get_logger().info('Exploration complete. All cells have been explored.')

        return response

    def update_costmap(self, msg):
        """Update global costmap."""
        self.costmap_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def goal_response_callback(self):
        """Inform user the goal pose was set."""
        self.get_logger().info('Navigation goal sent successfully.')


def main(args=None):
    """Init and run the Aitomatic Explore Node."""
    rclpy.init(args=args)
    node = AutomaticExplore()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '___main__':
    main()
