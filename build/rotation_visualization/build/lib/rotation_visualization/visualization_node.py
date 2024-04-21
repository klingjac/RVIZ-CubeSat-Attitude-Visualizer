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
import ast
import numpy as np
from .Kalman import QuaternionKalmanFilter
from .QUEST import quest
from .sun_vect import compute_vect

class CSVReaderNode(Node):
    def __init__(self):
        super().__init__('csv_reader_node')
        self.publisher = self.create_publisher(Quaternion, 'orientation', 10)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(1.0/37, self.timer_callback)
        self.csv_file_path = os.path.expanduser('~/AERO740/rotation_visualization_ws/rotation_visualization/data/concat_data2.csv')
        self.csv_data = self.read_csv(self.csv_file_path)
        self.data_index = 0
        self.model_pub = self.create_publisher(Marker, 'virtual_sat_model', 10)
        # self.marker_pub = self.create_publisher(Marker, 'cubesat_marker', 10)
        self.ekf = QuaternionKalmanFilter()

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
        mag_vect = row[17]
        vect = mag_vect.strip('[]')
        vect = vect.split()
        mag_vect = [float(num) for num in vect]

        mag_ref = row[18]

        vect = mag_ref.strip('[]')
        vect = vect.split()
        mag_ref = [float(num) for num in vect]

        sun_ref = row[19]

        vect = sun_ref.strip('[]')
        vect = vect.split()
        sun_ref = [float(num) for num in vect]

        t11 = float(row[7])
        t12 = float(row[8])
        t13 = float(row[9])
        t21 = float(row[10])
        t22 = float(row[11])
        t23 = float(row[12])

        gx = float(row[4])
        gy = float(row[5])
        gz = float(row[6])

        pannels = row[25]

        vect = pannels.strip('[]')
        vect = vect.split()
        pannels = [float(num) for num in vect]

        sun_vect = compute_vect(t11,t12,t13,t21,t22,t23,pannels)

        flat_sun_vect = sun_vect.flatten()
        body_vs = np.vstack((flat_sun_vect, mag_vect))
        ref_vs = np.vstack((sun_ref, mag_ref))
        weights = np.vstack((10,2))
        static_q = quest(body_vs, weights, ref_vs)
        


        gyro_vect = np.array([gx, gy, gz])

        
        self.ekf.predict(gyro_vect)
        self.ekf.update(static_q.flatten())

        dynamic_q = self.ekf.state

        self.get_logger().info(f'Quat: {dynamic_q}')

        # quat_str = row[24]
        # quat_str_cleaned = quat_str.strip('[]')
        # quat_parts = quat_str_cleaned.split()

        # # Convert each part to a float
        # q = [float(num) for num in quat_parts]
        #yaw, pitch, roll = float(row[0]), float(row[1]), float(row[2])
        
        

        quaternion_msg = Quaternion()
        quaternion_msg.w, quaternion_msg.x, quaternion_msg.y, quaternion_msg.z = dynamic_q.flatten()
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
        mesh.mesh_resource = "package://rotation_visualization/models/VirtualSatv8.stl"
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

        # marker = Marker()
        # marker.header.frame_id = "cubesat"  # Use the child frame ID
        # marker.header.stamp = self.get_clock().now().to_msg()
        
        # marker.type = marker.CUBE
        # marker.action = marker.ADD
        # marker.scale.x = 1.0  # Size of the marker [m]
        # marker.scale.y = 1.0
        # marker.scale.z = 3.0
        # marker.color.a = 1.0  # Alpha must be non-zero
        # marker.color.r = 1.0  # Red color
        # marker.color.g = 0.0
        # marker.color.b = 0.0
        # marker.pose.orientation.w = quaternion_msg.w  # Neutral orientation
        # marker.pose.orientation.x = quaternion_msg.x  # Position at the origin of the frame
        # marker.pose.orientation.y = quaternion_msg.y
        # marker.pose.orientation.z = quaternion_msg.z
        # marker.pose.position.x = 0.0  # Optional: Adjust if the marker has a specific position
        # marker.pose.position.y = 0.0
        # marker.pose.position.z = 0.0

        # self.marker_pub.publish(marker)

def main(args=None):
    time.sleep(1)
    rclpy.init(args=args)
    node = CSVReaderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
