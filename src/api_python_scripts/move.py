import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
import math


class MoveItGoalSender(Node):
    def __init__(self):
        super().__init__('moveit_goal_sender')

        self._client = ActionClient(self, MoveGroup, '/move_action')

        self.get_logger().info("Waiting for MoveIt action server...")
        self._client.wait_for_server()
        self.get_logger().info("Connected to MoveIt!")

    # ─────────────────────────────────────────────
    # Send goal
    # ─────────────────────────────────────────────
    def send_pose_goal(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        self.get_logger().info(
            f'Sending goal -> x:{x} y:{y} z:{z} yaw:{math.degrees(yaw):.1f}°'
        )

        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1

        # Target pose
        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.header.stamp = self.get_clock().now().to_msg()

        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z

        target.pose.orientation.x = qx
        target.pose.orientation.y = qy
        target.pose.orientation.z = qz
        target.pose.orientation.w = qw

        # ───── Position constraint ─────
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'link6'

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.05]  # 5 cm tolerance

        pos_constraint.constraint_region.primitives = [sphere]
        pos_constraint.constraint_region.primitive_poses = [target.pose]
        pos_constraint.weight = 1.0

        # ───── Orientation constraint ─────
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'base_link'
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

        # ───── Send goal ─────
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
            self.get_logger().info('✅ SUCCESS!')
        else:
            self.get_logger().error(f'❌ FAILED with error code: {error_code}')
            self._log_error_meaning(error_code)

    # ─────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────
    def euler_to_quaternion(self, roll, pitch, yaw):
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return x, y, z, w

    def _log_error_meaning(self, code):
        meanings = {
            -1: 'FAILURE',
            -2: 'PLANNING_FAILED',
            -4: 'INVALID_MOTION_PLAN',
            -6: 'CONTROL_FAILED',
            -10: 'START_STATE_IN_COLLISION',
            -12: 'GOAL_IN_COLLISION',
            -14: 'GOAL_CONSTRAINTS_VIOLATED',
            -15: 'INVALID_GROUP_NAME',
            -16: 'INVALID_GOAL_CONSTRAINTS',
            -17: 'INVALID_ROBOT_STATE',
            99999: 'IK FAILURE / OUT OF REACH',
        }

        self.get_logger().error(meanings.get(code, 'UNKNOWN ERROR'))


# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────
def main():
    rclpy.init()

    node = MoveItGoalSender()

    # 🔥 CHANGE THIS based on your workspace
    node.send_pose_goal(
        x=0.1,
        y=0.1,
        z=0.9,
        roll=0.0,
        pitch=0.0,
        yaw=0.0
    )

    rclpy.shutdown()


if __name__ == '__main__':
    main()