import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
import tf2_ros
import tf2_geometry_msgs
import math


class MoveItClient(Node):
    def __init__(self):
        super().__init__('moveit_client')
        self._client = ActionClient(self, MoveGroup, '/move_action')

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher — broadcasts gripper pose in world frame
        self.gripper_pose_pub = self.create_publisher(
            PoseStamped,
            '/gripper_current_pose',
            10
        )

        # Timer — publishes gripper pose every 100ms
        self.timer = self.create_timer(0.1, self.publish_gripper_pose)

        self.get_logger().info('MoveItClient node started!')
        self.get_logger().info('Publishing gripper pose on /gripper_current_pose')

    # ─────────────────────────────────────────────
    # Continuously publish gripper pose in world frame
    # ─────────────────────────────────────────────
    def publish_gripper_pose(self):
        try:
            # link6 position in world frame
            transform = self.tf_buffer.lookup_transform(
                'world',
                'link6',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

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

            # Also get base_link position in world for reference
            base_transform = self.tf_buffer.lookup_transform(
                'world',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # Compute gripper position relative to base_link
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

    # ─────────────────────────────────────────────
    # Send goal in world frame
    # ─────────────────────────────────────────────
    def send_pose_goal(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        self.get_logger().info(
            f'Sending goal [world frame] -> '
            f'x:{x} y:{y} z:{z} '
            f'yaw:{math.degrees(yaw):.1f}°'
        )

        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1

        # Goal in world frame
        target = PoseStamped()
        target.header.frame_id = 'world'          # ← world frame
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        target.pose.orientation.x = qx
        target.pose.orientation.y = qy
        target.pose.orientation.z = qz
        target.pose.orientation.w = qw

        # Position constraint in world frame
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'world'  # ← world frame
        pos_constraint.link_name = 'link6'

        bounding_box = SolidPrimitive()
        bounding_box.type = SolidPrimitive.SPHERE
        bounding_box.dimensions = [0.05]           # 5cm tolerance

        pos_constraint.constraint_region.primitives = [bounding_box]
        pos_constraint.constraint_region.primitive_poses = [target.pose]
        pos_constraint.weight = 1.0

        # Orientation constraint in world frame
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'world'  # ← world frame
        ori_constraint.link_name = 'link6'
        ori_constraint.orientation = target.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.2
        ori_constraint.absolute_y_axis_tolerance = 0.2
        ori_constraint.absolute_z_axis_tolerance = 0.2
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [pos_constraint]
        constraints.orientation_constraints = [ori_constraint]
        goal.request.goal_constraints = [constraints]

        # Send goal
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        error_code = result_future.result().result.error_code.val
        if error_code == 1:
            self.get_logger().info('SUCCESS!')
        else:
            self.get_logger().error(f'FAILED with error code: {error_code}')
            self._log_error_meaning(error_code)

    # ─────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────
    def euler_to_quaternion(self, roll, pitch, yaw):
        cy, sy = math.cos(yaw / 2),   math.sin(yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cr, sr = math.cos(roll / 2),  math.sin(roll / 2)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return x, y, z, w

    def _log_error_meaning(self, code):
        meanings = {
            -1:    'FAILURE — generic failure',
            -2:    'PLANNING_FAILED — no valid path found',
            -4:    'INVALID_MOTION_PLAN',
            -6:    'CONTROL_FAILED',
            -10:   'START_STATE_IN_COLLISION',
            -12:   'GOAL_IN_COLLISION',
            -14:   'GOAL_CONSTRAINTS_VIOLATED',
            -15:   'INVALID_GROUP_NAME',
            -16:   'INVALID_GOAL_CONSTRAINTS',
            -17:   'INVALID_ROBOT_STATE',
            99999: 'UNDEFINED — IK solver failure or goal out of workspace',
        }
        meaning = meanings.get(code, 'Unknown error code')
        self.get_logger().error(f'Error meaning: {meaning}')


def main():
    rclpy.init()
    node = MoveItClient()

    # Warm up TF buffer
    node.get_logger().info('Warming up TF buffer...')
    start = node.get_clock().now()
    while (node.get_clock().now() - start).nanoseconds < 2e9:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info('TF buffer ready!')

    # Send goal in world frame
    # Use the world frame coordinates from your gripper pose printout
    node.send_pose_goal(
        x=0.334,    # world frame x
        y=0.319,    # world frame y
        z=1.075,    # world frame z
        roll=0.0,
        pitch=0.0,
        yaw=0.0
    )

    # Keep spinning
    node.get_logger().info('Spinning... publishing gripper pose on /gripper_current_pose')
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
