import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
import math


class MoveItClient(Node):
    def __init__(self):
        super().__init__('moveit_client')
        self._client = ActionClient(self, MoveGroup, '/move_action')

        # Publisher — goal pose arrow in RViz
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose_viz',
            10
        )

        # Publisher — goal marker (sphere) in RViz
        self.marker_pub = self.create_publisher(
            Marker,
            '/goal_marker',
            10
        )

        self.get_logger().info('MoveIt Client started!')

    # ─────────────────────────────────────────────
    # Publish goal visualization to RViz
    # ─────────────────────────────────────────────
    def publish_goal_viz(self, x, y, z, qx, qy, qz, qw):

        # 1. PoseStamped — shows as arrow in RViz
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.goal_pub.publish(pose_msg)

        # 2. Marker sphere — shows as ball at goal position
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'world'
        marker.ns = 'goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05   # 5cm sphere
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0    # red
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8    # slightly transparent
        self.marker_pub.publish(marker)

        # 3. Arrow marker — shows orientation
        arrow = Marker()
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.header.frame_id = 'world'
        arrow.ns = 'goal_arrow'
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = x
        arrow.pose.position.y = y
        arrow.pose.position.z = z
        arrow.pose.orientation.x = qx
        arrow.pose.orientation.y = qy
        arrow.pose.orientation.z = qz
        arrow.pose.orientation.w = qw
        arrow.scale.x = 0.1    # arrow length
        arrow.scale.y = 0.01   # arrow width
        arrow.scale.z = 0.01   # arrow height
        arrow.color.r = 0.0
        arrow.color.g = 1.0    # green
        arrow.color.b = 0.0
        arrow.color.a = 0.8
        self.marker_pub.publish(arrow)

        self.get_logger().info('Goal visualization published to RViz!')

    # ─────────────────────────────────────────────
    # Send goal to MoveIt
    # ─────────────────────────────────────────────
    def send_pose_goal(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        self.get_logger().info(
            f'Sending goal [world frame] -> '
            f'x:{x} y:{y} z:{z} '
            f'roll:{math.degrees(roll):.1f}° '
            f'pitch:{math.degrees(pitch):.1f}° '
            f'yaw:{math.degrees(yaw):.1f}°'
        )

        self._client.wait_for_server()
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

        # Publish visualization BEFORE sending goal
        self.publish_goal_viz(x, y, z, qx, qy, qz, qw)

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1

        target = PoseStamped()
        target.header.frame_id = 'world'
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        target.pose.orientation.x = qx
        target.pose.orientation.y = qy
        target.pose.orientation.z = qz
        target.pose.orientation.w = qw

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'world'
        pos_constraint.link_name = 'link6'

        bounding_box = SolidPrimitive()
        bounding_box.type = SolidPrimitive.SPHERE
        bounding_box.dimensions = [0.05]

        pos_constraint.constraint_region.primitives = [bounding_box]
        pos_constraint.constraint_region.primitive_poses = [target.pose]
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'world'
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

    # ── Set your goal here ────────────────────────
    node.send_pose_goal(
        x=0.1,
        y=0.55,
        z=1.0,
        roll=1.0,
        pitch=3.14,
        yaw=0.0
    )
    # ─────────────────────────────────────────────

    # Keep spinning so marker stays visible in RViz
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

    # node.send_pose_goal(
    #     x=0.1,
    #     y=0.55,
    #     z=1.0,
    #     roll=0.0,
    #     pitch=-1.57,
    #     yaw=0.0