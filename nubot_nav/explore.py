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

        # Identify unexplored cells (-1 values)
        unexplored_cells = np.argwhere(self.costmap_data == -1)

        # Apply a safety buffer: Exclude cells near obstacles or boundaries
        inflated_map = np.copy(self.costmap_data)
        inflation_radius = 2  # Adjust based on costmap resolution (2 cells = 10 cm for 0.05 resolution)

        # Inflate the obstacles by setting nearby cells to a high value
        for cell_y, cell_x in np.argwhere(self.costmap_data >= 100):  # Obstacles are often >=100 in costmaps
            for dy in range(-inflation_radius, inflation_radius + 1):
                for dx in range(-inflation_radius, inflation_radius + 1):
                    ny, nx = cell_y + dy, cell_x + dx
                    if 0 <= ny < inflated_map.shape[0] and 0 <= nx < inflated_map.shape[1]:
                        inflated_map[ny, nx] = 100

        # Filter unexplored cells to exclude those within the inflated obstacle zone
        safe_unexplored_cells = []
        for cell_y, cell_x in unexplored_cells:
            if inflated_map[cell_y, cell_x] < 100:
                safe_unexplored_cells.append((cell_y, cell_x))

        if len(safe_unexplored_cells) == 0:
            self.logger.info('No safe unexplored areas.')
            return np.array([])

        return np.array(safe_unexplored_cells)


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