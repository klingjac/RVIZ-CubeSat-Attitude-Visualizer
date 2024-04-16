import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
import tf2_ros
import csv
import os
import math
import time
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation as R

class CSVReaderNode(Node):
    def __init__(self):
        super().__init__('csv_reader_node')
        self.publisher = self.create_publisher(Quaternion, 'orientation', 10)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(1.0/8, self.timer_callback)
        self.csv_file_path = os.path.expanduser('~/AERO740/rotation_visualization_ws/rotation_visualization/data/ADS_3.csv')
        self.csv_data = self.read_csv(self.csv_file_path)
        self.data_index = 0
        self.model_pub = self.create_publisher(Marker, 'virtual_sat_model', 10)

    def read_csv(self, file_path):
        try:
            with open(file_path, newline='') as csvfile:
                data_reader = csv.reader(csvfile, delimiter=',')
                next(data_reader, None)  # Skip header
                return [row for row in data_reader]
        except FileNotFoundError:
            self.get_logger().error(f"File not found: {file_path}")
            return []

    def timer_callback(self):
        if self.data_index >= len(self.csv_data):
            self.get_logger().info('End of CSV data reached.')
            return

        row = self.csv_data[self.data_index]
        yaw, pitch, roll = float(row[0]), float(row[1]), float(row[2])
        q = [float(row[4]), float(row[5]), float(row[6]), float(row[3])]

        quaternion_msg = Quaternion()
        quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w = q
        self.publisher.publish(quaternion_msg)
        self.data_index += 1

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'virtual_sat'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = quaternion_msg

        self.br.sendTransform(t)
        self.publish_model(quaternion_msg)

    def publish_model(self, quaternion_msg):
        mesh = Marker()
        mesh.header.frame_id = "virtual_sat"
        mesh.header.stamp = self.get_clock().now().to_msg()
        mesh.type = mesh.MESH_RESOURCE
        mesh.mesh_resource = "package://rotation_visualization/models/VirtualSatv6.stl"
        mesh.pose.orientation = quaternion_msg
        mesh.pose.position.x = 0.0
        mesh.pose.position.y = 0.0
        mesh.pose.position.z = 0.0
        mesh.scale.x = 1.0/100  # Adjust scale as needed
        mesh.scale.y = 1.0/100
        mesh.scale.z = 1.0/100
        mesh.color.a = 1.0  # Alpha must be non-zero
        mesh.color.r = 1.0  # Red color
        mesh.color.g = 0.0
        mesh.color.b = 0.0

        self.model_pub.publish(mesh)

def main(args=None):
    time.sleep(10)
    rclpy.init(args=args)
    node = CSVReaderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
