import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
import numpy as np
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid


class AutomaticExplore(Node):
    """Ros2 Node designed to implement single exploration."""

    def __init__(self):
        """Create AutomaticExplore Node."""
        super().__init__('explore_node')
        self.get_logger().info('Automatic Explore Node Started!')
        self.logger = self.get_logger()

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.VOLATILE)

        # Costmap subscription
        self.costmap_subscriber = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.update_costmap, qos)

        # Service to start navigation
        self.create_service(Empty, 'start_navigation', self.start_navigation)

        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation action server not available.")
            return
        self.get_logger().info('Navigator is ready.')

        # Subscribe to robot pose
        self.robot_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.update_robot_pose, qos)

        self.costmap_data = None
        self.map_info = None

    def get_unexplored_cells(self):
        """Get unexplored cells from the costmap."""
        if self.costmap_data is None:
            return np.array([])
        unexplored_cell = np.argwhere(self.costmap_data == -1)
        if unexplored_cell.size == 0:
            self.logger.info('No more unexplored areas.')
        return unexplored_cell

    def random_unexplored_target(self, unexplored_cells):
        """Select a random unexplored cell."""
        if unexplored_cells.size == 0:
            return None
        random_index = np.random.choice(len(unexplored_cells))
        cell_y, cell_x = unexplored_cells[random_index]
        target_x = self.map_info.origin.position.x + (cell_x + 0.5) * self.map_info.resolution
        target_y = self.map_info.origin.position.y + (cell_y + 0.5) * self.map_info.resolution
        return {'x': target_x, 'y': target_y}

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
        # rclpy.spin_until_future_complete(self, future)

        # goal_handle = future.result()
        # if not goal_handle.accepted:
        #     self.get_logger().warning('Goal was rejected by the action server.')
        #     return False

        self.get_logger().info('Goal accepted. Waiting for result...')

        # Wait for the result
        # result_future = goal_handle.get_result_async()
        # rclpy.spin_until_future_complete(self, result_future)

        # result = result_future.result()
        # if result.status == 4:  # SUCCEEDED
        #     self.get_logger().info('Target goal reached')
        #     return True
        # else:
        #     self.get_logger().warning(f'Goal failed with status: {result.status}')
        #     return False

    def start_navigation(self, request, response):
        """Start Single Navigation."""
        if self.costmap_data is None or self.map_info is None:
            self.get_logger().error("Costmap or map information is not available.")
            response.success = False
            return response

        # Get unexplored cells
        unexplored_cells = self.get_unexplored_cells()
        
        # Select a random target
        target = self.random_unexplored_target(unexplored_cells)
        
        if target is None:
            self.get_logger().info('No valid unexplored target found.')
            response.success = False
            return response

        # Move to the target
        self.move_to_target(target)
        
        
        self.get_logger().info('Single exploration attempt complete.')
        return response

    def update_costmap(self, msg):
        """Update global costmap."""
        if not msg.data:
            self.get_logger().warning("Received an empty costmap update.")
            return
        self.costmap_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def update_robot_pose(self, msg):
        """Update the robot's current pose."""
        self.robot_pose = msg.pose.pose


def main(args=None):
    """Init and run the Automatic Explore Node."""
    rclpy.init(args=args)
    node = AutomaticExplore()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()