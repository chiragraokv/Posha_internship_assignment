import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from ament_index_python.packages import get_package_share_directory
import tf2_ros
import math
import yaml
import os
import time


class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')

        # MoveIt action client
        self._client = ActionClient(self, MoveGroup, '/move_action')

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Declare parameter for yaml file path
        self.declare_parameter(
            'waypoints_file',
            os.path.join(
                get_package_share_directory('piper_automation'),
                'config',
                'waypoints.yaml'
            )
        )

        self.get_logger().info('Pick and Place node started!')

    # ─────────────────────────────────────────────
    # Load waypoints from yaml
    # ─────────────────────────────────────────────
    def load_waypoints(self):
        waypoints_file = self.get_parameter('waypoints_file').value
        self.get_logger().info(f'Loading waypoints from: {waypoints_file}')

        with open(waypoints_file, 'r') as f:
            data = yaml.safe_load(f)

        waypoints = data['waypoints']
        self.get_logger().info(f'Loaded {len(waypoints)} waypoints')
        return waypoints

    # ─────────────────────────────────────────────
    # Execute all waypoints in sequence
    # ─────────────────────────────────────────────
    def execute(self):
        waypoints = self.load_waypoints()

        self.get_logger().info('Starting pick and place sequence...')
        self.get_logger().info('─' * 40)

        for i, wp in enumerate(waypoints):
            name    = wp['name']
            pos     = wp['position']
            ori     = wp['orientation']
            gripper = wp['gripper']
            action  = wp['action']

            self.get_logger().info(
                f'[{i+1}/{len(waypoints)}] '
                f'Waypoint: "{name}" | '
                f'action: {action} | '
                f'gripper: {gripper}'
            )

            # Move arm to waypoint
            success = self.send_pose_goal(
                x=pos['x'],
                y=pos['y'],
                z=pos['z'],
                roll=ori['roll'],
                pitch=ori['pitch'],
                yaw=ori['yaw']
            )

            if not success:
                self.get_logger().error(
                    f'Failed at waypoint "{name}" — stopping sequence!'
                )
                return

            # Control gripper
            self.control_gripper(gripper)

            # Small delay between waypoints
            time.sleep(0.5)
            self.get_logger().info('─' * 40)

        self.get_logger().info('Pick and place sequence COMPLETE!')

    # ─────────────────────────────────────────────
    # Send pose goal to MoveIt
    # ─────────────────────────────────────────────
    def send_pose_goal(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        self.get_logger().info(
            f'Moving to -> x:{x:.3f} y:{y:.3f} z:{z:.3f} '
            f'roll:{math.degrees(roll):.1f}° '
            f'pitch:{math.degrees(pitch):.1f}° '
            f'yaw:{math.degrees(yaw):.1f}°'
        )

        self._client.wait_for_server()
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

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
            return False

        self.get_logger().info('Goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        error_code = result_future.result().result.error_code.val
        if error_code == 1:
            self.get_logger().info('Reached waypoint!')
            return True
        else:
            self.get_logger().error(f'FAILED with error code: {error_code}')
            self._log_error_meaning(error_code)
            return False

    # ─────────────────────────────────────────────
    # Gripper control
    # ─────────────────────────────────────────────
    def control_gripper(self, state):
        self.get_logger().info(f'Gripper -> {state.upper()}')

        self._client.wait_for_server()

        goal = MoveGroup.Goal()
        goal.request.group_name = 'gripper'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5

        from moveit_msgs.msg import JointConstraint
        jc = JointConstraint()
        jc.joint_name = 'joint7'
        jc.position = 0.035 if state == 'open' else 0.0
        jc.tolerance_above = 0.005
        jc.tolerance_below = 0.005
        jc.weight = 1.0

        constraints = Constraints()
        constraints.joint_constraints = [jc]
        goal.request.goal_constraints = [constraints]

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected!')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        code = result_future.result().result.error_code.val
        if code == 1:
            self.get_logger().info(f'Gripper {state} SUCCESS!')
        else:
            self.get_logger().error(f'Gripper FAILED: {code}')

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
            99999: 'UNDEFINED — IK solver failure or out of workspace',
        }
        self.get_logger().error(meanings.get(code, 'Unknown error'))


def main():
    rclpy.init()
    node = PickAndPlace()

    # Warm up TF buffer
    node.get_logger().info('Warming up TF buffer...')
    start = node.get_clock().now()
    while (node.get_clock().now() - start).nanoseconds < 2e9:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info('TF buffer ready!')

    # Run sequence
    node.execute()

    rclpy.shutdown()


if __name__ == '__main__':
    main()