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
from .EKFQb import QuaternionKalmanFilter
from .QUEST import quest
from .sun_vect import compute_vect
import ast
import numpy as np
from datetime import datetime
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
from datetime import datetime
import tf_transformations

class CSVReaderNode(Node):
    def __init__(self):
        super().__init__('csv_reader_node')
        self.publisher = self.create_publisher(Quaternion, 'orientation', 10)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(1.0/37, self.timer_callback)
        self.csv_file_path = os.path.expanduser('~/AERO740/rotation_visualization_ws/rotation_visualization/data/combined_ADS.csv')
        self.csv_data = self.read_csv(self.csv_file_path)
        self.data_index = 0
        self.model_pub = self.create_publisher(Marker, 'virtual_sat_model', 10)
        self.marker_pub = self.create_publisher(Marker, 'sun_marker', 10)
        self.grav_pub = self.create_publisher(Marker, 'acc_vect', 10)

        self.ekf = QuaternionKalmanFilter()

        self.dynamic_quaternions = []
        self.times = []
        self.mags = []

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
        csv_time = float(row[0])
        self.times.append(csv_time)
        quat_str = row[27]
        quat_str_cleaned = quat_str.strip('[]')
        quat_parts = quat_str_cleaned.split()

        t11 = float(row[10])
        t12 = float(row[11])
        t13 = float(row[12])
        t21 = float(row[13])
        t22 = float(row[14])
        t23 = float(row[15])

        gx = float(row[4])
        gy = float(row[5])
        gz = float(row[6])

        ax = float(row[7])
        ay = float(row[8])
        az = float(row[9])

        acc_vect = [ax, ay, az]
        print(f"Current ACC VECT: {acc_vect}")

        mag_vect = row[20]
        magx = float(row[1])
        magy = float(row[2])
        magz = float(row[3])
        mg = [magx, magy, magz]
        self.mags.append(mg)
        vect = mag_vect.strip('[]')
        vect = vect.split()
        mag_vect = [float(num) for num in vect]

        mag_ref = row[21]

        vect = mag_ref.strip('[]')
        vect = vect.split()
        #mag_ref = [float(num) for num in vect]
        mag_ref = [ 0.04348156,  0.6488787,  -0.75964847]

        sun_ref = row[22]

        vect = sun_ref.strip('[]')
        vect = vect.split()
        #sun_ref = [float(num) for num in vect]
        sun_ref = [0.10519216, 0.2465034,  0.96341615]

        # Convert each part to a float
        q = [float(num) for num in quat_parts]
        #yaw, pitch, roll = float(row[0]), float(row[1]), float(row[2])
        self.get_logger().info(f'Quat: {quat_str}')

        sun_vect = compute_vect(t11,t12,t13,t21,t22,t23) #compute the sun vector

        flat_sun_vect = sun_vect.flatten()
        body_vs = np.vstack((flat_sun_vect, mag_vect))
        ref_vs = np.vstack((sun_ref, mag_ref))

        weights = np.vstack((1,1))
        static_q = quest(body_vs, weights, ref_vs)
        print(f"this: {static_q}")
        static_q = static_q.flatten()
        print(f"that: {static_q}")

        gyro_vect = np.array([gx, gy, gz])

        
        self.ekf.predict(gyro_vect)
        self.ekf.update(static_q.flatten(), gyro_vect)

        dynamic_q = self.ekf.state

        
        

        quaternion_msg = Quaternion()
        quaternion_msg.w, quaternion_msg.x, quaternion_msg.y, quaternion_msg.z = dynamic_q[:4]
        #quaternion_msg.w, quaternion_msg.x, quaternion_msg.y, quaternion_msg.z = static_q
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
        self.publish_model(quaternion_msg, flat_sun_vect, acc_vect)

    def vector_to_quaternion(self, vector):
        # Normalize the vector
        norm = np.linalg.norm(vector)
        vector2 = vector / norm
        print(f"vect2: {vector2}")
        q = [float(0), vector2[0], vector2[1], vector2[2]]
        return q

    def quaternion_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        return [w, x, y, z]

    def publish_model(self, quaternion_msg, sun_v, acc_vect):
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

        marker = Marker()
        marker.header.frame_id = "virtual_sat"  # Use the child frame ID
        marker.header.stamp = mesh.header.stamp
        
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 1.0  # Size of the marker [m]
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0  # Alpha must be non-zero
        marker.color.r = 0.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.pose.orientation.w = quaternion_msg.w  # Neutral orientation
        marker.pose.orientation.x = quaternion_msg.x  # Position at the origin of the frame
        marker.pose.orientation.y = quaternion_msg.y
        marker.pose.orientation.z = quaternion_msg.z

        q = [quaternion_msg.w, quaternion_msg.x, quaternion_msg.y, quaternion_msg.z]

        marker.pose.position.x = 3*float(sun_v[0])  # Optional: Adjust if the marker has a specific position
        marker.pose.position.y = 3*float(sun_v[1])
        marker.pose.position.z = 3*float(sun_v[2])

        self.marker_pub.publish(marker)

        marker = Marker()
        marker.header.frame_id = "virtual_sat"  # Use the child frame ID
        marker.header.stamp = mesh.header.stamp

        marker.type = Marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 1.0  # Size of the marker [m]
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0  # Alpha must be non-zero
        marker.color.r = 0.0  # Red color
        marker.color.g = 1.0
        marker.color.b = 0.0

        pose = self.vector_to_quaternion(acc_vect)
        rotation_quaternion = [-np.sqrt(2)/2, 0, np.sqrt(2)/2, 0]
        visual_quat = self.quaternion_multiply(rotation_quaternion, pose)
        print(f"pose: {pose}")

        marker.pose.orientation.w = visual_quat[0]  # Neutral orientation
        marker.pose.orientation.x = visual_quat[1]  # Position at the origin of the frame
        marker.pose.orientation.y = visual_quat[2]
        marker.pose.orientation.z = visual_quat[3]

        marker.pose.position.x = float(0)
        marker.pose.position.y = float(0)
        marker.pose.position.z = float(-2)

        self.grav_pub.publish(marker)

        
       

        

    def plot_quaternions(self):
        # Convert lists to numpy arrays for easier slicing
        quaternions = np.array(self.mags)
        print(quaternions)
        # times = np.array(self.times)
        times = self.times
        window_size = 10  # Adjust based on your dataset and preferences

        # Calculate moving averages for each component
        x = np.array(quaternions[:, 0])
        y = np.array(quaternions[:, 1])
        z = np.array(quaternions[:, 2])
        # times = (times - times[0]) * 1e-9  # Convert to seconds from nanoseconds and zero the time
        magnitude = np.sqrt(x**2 + y**2 + z**2)
        plt.figure()
        plt.plot(times, x, label='x')
        plt.plot(times, y, label='y')
        plt.plot(times, z, label='z')
        plt.plot(times, magnitude, label='total magnitude')
        
        plt.xlabel('Time (s)')
        plt.ylabel('Magnetic field')
        plt.title('Magnetic field values')
        plt.legend()
        plt.show()

        # plt.figure(figsize=(10, 8))
        # plt.plot(times[window_size-1:], q0_avg, label='q0 (w) avg', linewidth=1)
        # plt.plot(times[window_size-1:], q1_avg, label='q1 (x) avg', linewidth=1)
        # plt.plot(times[window_size-1:], q2_avg, label='q2 (y) avg', linewidth=1)
        # plt.plot(times[window_size-1:], q3_avg, label='q3 (z) avg', linewidth=1)
        # plt.xlabel('Time (s)')
        # plt.ylabel('Moving Average of Quaternion Components')
        # plt.title('Moving Averages of Quaternion Components Over Time')
        # plt.legend()
        # plt.grid(True)
        # plt.show()

def main(args=None):
    time.sleep(1)
    rclpy.init(args=args)
    node = CSVReaderNode()
    
    # The rclpy.spin() will block execution until the node is shutdown
    # The node will be shutdown after processing all the data or if the user interrupts the process
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    # except BaseException:
    #     node.get_logger().info('Exception in CSVReaderNode:', file=sys.stderr)
    #     raise
        
    finally:
    
        node.plot_quaternions()
        node.destroy_node()
    # After spinning and processing the CSV data, we plot the quaternion data
        # node.plot_quaternions()

if __name__ == '__main__':
    main()
