from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh
from geometry_msgs.msg import Pose
from moveit_commander import PlanningSceneInterface
import rclpy
from rclpy.node import Node
from stl import mesh
import numpy as np

class StoveAdder(Node):
    def __init__(self):
        super().__init__('stove_adder')
        self.scene = PlanningSceneInterface()

        self.add_stove()

    def add_stove(self):
        co = CollisionObject()
        co.id = "stove"
        co.header.frame_id = "world"   # 🔴 IMPORTANT

        # Load STL
        stove_mesh = mesh.Mesh.from_file(
            '/absolute/path/to/stove_simple.stl'
        )

        mesh_msg = Mesh()
        vertices = stove_mesh.vectors.reshape(-1, 3)

        for v in vertices:
            from geometry_msgs.msg import Point
            p = Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))
            mesh_msg.vertices.append(p)

        for i in range(0, len(vertices), 3):
            from shape_msgs.msg import MeshTriangle
            tri = MeshTriangle(vertex_indices=[i, i+1, i+2])
            mesh_msg.triangles.append(tri)

        co.meshes.append(mesh_msg)

        pose = Pose()
        pose.position.x = 0.435   # 🔴 adjust this!
        pose.position.y = 0.84
        pose.position.z = 0.0     # likely ground

        pose.orientation.w = 1.0

        co.mesh_poses.append(pose)
        co.operation = CollisionObject.ADD

        self.scene.apply_collision_object(co)
        self.get_logger().info("Stove added")

def main():
    rclpy.init()
    node = StoveAdder()
    rclpy.spin(node)
    rclpy.shutdown()