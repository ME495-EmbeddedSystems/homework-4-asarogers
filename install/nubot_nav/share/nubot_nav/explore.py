import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
import numpy as np
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallFollowingRobot(Node):
    """ROS2 Node for left wall following behavior."""

    def __init__(self):
        """Create WallFollowingRobot Node."""
        super().__init__('wall_following_node')
        self.get_logger().info('Wall Following Node Started!')
        self.logger = self.get_logger().info

        # QoS Profile for laser scan subscription
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.VOLATILE)

        # Subscribe to laser scan topic
        self.laser_subscriber = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos)

        # Create publisher for cmd_vel to control robot movement
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Wall following parameters
        self.target_distance = 0.5  # Target distance from the wall (meters)
        self.kp_distance = 0.5  # Proportional gain for distance error
        self.kp_angular = 0.5  # Proportional gain for angular correction

    # def get_sector_indices(self, start_deg, end_deg, total_points):
    #     """
    #     Convert degrees to indices for a laser scan array.

    #     :param start_deg: Start angle in degrees.
    #     :param end_deg: End angle in degrees.
    #     :param total_points: Total points in the laser scan array.
    #     :return: Start and end indices corresponding to the given angles.
    #     """
    #     start_idx = int((start_deg / 360.0) * total_points)
    #     end_idx = int((end_deg / 360.0) * total_points)
    #     return start_idx, end_idx

    def get_sector_distances(self, laser_ranges):
        """
        Compute sector distances from laser scan ranges based on degree-defined sectors.

        Sectors:
        - Left: 270° to 350°
        - Front Left: 45° to 90°
        - Front Right: 315° to 360°
        """

        rear_sector = laser_ranges[0:90]
        left_sector = laser_ranges[400:440]
        front = laser_ranges[300:330]
        right_sector = laser_ranges[500:530]

        return {
            'rear_sector': min(rear_sector),
            'left_sector': min(left_sector),
            'front': min(front),
            'right_sector': min(right_sector)
        }

    def laser_callback(self, msg):
        """Process laser scan data and compute wall following behavior."""
        # Filter out inf and nan values
        ranges = np.array(msg.ranges)
        self.logger(str(ranges.size))
        # Get sector distances
        distances = self.get_sector_distances(ranges)

        # Compute control commands
        twist = Twist()

        # move forward
        linear = 1.0

        if distances['front'] < 4.5:
            linear = 0.0  # Stop
            twist.angular.z = -1.5  # Rotate to avoid obstacle

        if distances['front'] > 4.5 and distances['left_sector'] < 2.0:
            twist.angular.z = -1.0
            linear = 0.0


        
        # Velocity limits
        twist.linear.x = max(linear, -0.3)
        # twist.angular.z = max(min(twist.angular.z, 0.5), -0.5)

        # Publish velocity commands
        self.cmd_publisher.publish(twist)
        
        # Log for debugging
        self.get_logger().info(
            # f'Left: {distances["left_min"]:.2f}m, '
            # f'Front Left: {distances["front_left_min"]:.2f}m, '
            # f'rear_sector: {distances["rear_sector"]:.2f}m, '
            f'left_sector: {distances["left_sector"]:.2f}m, '
            f'front: {distances["front"]:.2f}m, '
            # f'right_sector: {distances["right_sector"]:.2f}m, '
            # f'Linear: {twist.linear.x:.2f}, Angular: {twist.angular.z:.2f}'
        )


def main(args=None):
    """Initialize and run the Wall Following Node."""
    rclpy.init(args=args)
    node = WallFollowingRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
