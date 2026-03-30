import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import math


class GripperMonitor(Node):
    def __init__(self):
        super().__init__('gripper_monitor')

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher
        self.gripper_pose_pub = self.create_publisher(
            PoseStamped,
            '/gripper_current_pose',
            10
        )

        # Timer — 10Hz
        self.timer = self.create_timer(0.1, self.publish_gripper_pose)

        self.get_logger().info('Gripper Monitor started!')
        self.get_logger().info('Publishing on /gripper_current_pose')

    def publish_gripper_pose(self):
        try:
            # link6 in world frame
            transform = self.tf_buffer.lookup_transform(
                'world',
                'link6',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # base_link in world frame
            base_transform = self.tf_buffer.lookup_transform(
                'world',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # Publish world frame pose
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.pose.position.x = transform.transform.translation.x
            msg.pose.position.y = transform.transform.translation.y
            msg.pose.position.z = transform.transform.translation.z
            msg.pose.orientation.x = transform.transform.rotation.x
            msg.pose.orientation.y = transform.transform.rotation.y
            msg.pose.orientation.z = transform.transform.rotation.z
            msg.pose.orientation.w = transform.transform.rotation.w
            self.gripper_pose_pub.publish(msg)

            # Gripper relative to base_link
            gx = transform.transform.translation.x - base_transform.transform.translation.x
            gy = transform.transform.translation.y - base_transform.transform.translation.y
            gz = transform.transform.translation.z - base_transform.transform.translation.z

            self.get_logger().info(
                f'\n'
                f'  [world frame]     x:{transform.transform.translation.x:.3f} '
                f'y:{transform.transform.translation.y:.3f} '
                f'z:{transform.transform.translation.z:.3f}\n'
                f'  [base_link frame] x:{gx:.3f} '
                f'y:{gy:.3f} '
                f'z:{gz:.3f}',
                throttle_duration_sec=1.0
            )

        except Exception as e:
            self.get_logger().warn(
                f'Could not get gripper pose: {e}',
                throttle_duration_sec=2.0
            )


def main():
    rclpy.init()
    node = GripperMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()