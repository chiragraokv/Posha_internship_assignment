import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import Mesh, MeshTriangle
from std_msgs.msg import Header
import numpy as np
from stl import mesh as stl_mesh
from geometry_msgs.msg import Point

class SceneSetup(Node):
    def __init__(self):
        super().__init__('scene_setup')
        self.pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        import time
        time.sleep(1.0)
        self.setup_scene()

    def load_mesh(self, path, scale=1.0):
        
        
        stl_data = stl_mesh.Mesh.from_file(path)
        mesh = Mesh()

        vertex_index = 0
        for triangle in stl_data.vectors:
            tri = MeshTriangle()
            tri.vertex_indices = [vertex_index, vertex_index + 1, vertex_index + 2]
            mesh.triangles.append(tri)

            for vertex in triangle:
                p = Point()
                p.x = float(vertex[0]) * scale
                p.y = float(vertex[1]) * scale
                p.z = float(vertex[2]) * scale
                mesh.vertices.append(p)

            vertex_index += 3

        return mesh

    def setup_scene(self):
        scene = PlanningScene()
        scene.is_diff = True

        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = 'stove'

        # Load your STL
        mesh = self.load_mesh(
            path='src/piper_description/meshes/stove_simple.stl',  # change this to your STL path
            scale=0.001  # change to 0.001 if your STL is in mm
        )

        pose = Pose()
        pose.position.x = 0.0   # adjust position relative to robot base
        pose.position.y = 0.0
        pose.position.z = -0.70
        pose.orientation.w = 1.0

        obj.meshes = [mesh]
        obj.mesh_poses = [pose]
        obj.operation = CollisionObject.ADD

        scene.world.collision_objects.append(obj)
        self.pub.publish(scene)
        self.get_logger().info('Stove mesh added to scene!')

def main():
    rclpy.init()
    node = SceneSetup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()