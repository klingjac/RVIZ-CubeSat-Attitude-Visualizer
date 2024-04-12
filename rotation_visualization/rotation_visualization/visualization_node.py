import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped
import tf2_ros
import csv
import os
from visualization_msgs.msg import Marker


class CSVReaderNode(Node):
    def __init__(self):
        super().__init__('csv_reader_node')
        # Publishers
        self.publisher = self.create_publisher(Quaternion, 'orientation', 10)
        # TF Broadcaster
        self.br = tf2_ros.TransformBroadcaster(self)
        # Timer setup to call timer_callback every second
        self.timer = self.create_timer(1.0/10, self.timer_callback)
        # CSV file path setup and read data
        self.csv_file_path = os.path.expanduser('~/AERO740/rotation_visualization_ws/rotation_visualization/data/ADS.csv')
        self.csv_data = self.read_csv(self.csv_file_path)
        self.data_index = 0
        self.marker_pub = self.create_publisher(Marker, 'cubesat_marker', 10)


    def read_csv(self, file_path):
        """Read CSV file and return list of rows."""
        try:
            with open(file_path, newline='') as csvfile:
                data_reader = csv.reader(csvfile, delimiter=',')
                next(data_reader, None)  # Skip header
                return [row for row in data_reader]
        except FileNotFoundError:
            self.get_logger().error(f"File not found: {file_path}")
            return []

    def timer_callback(self):
        """Callback function that runs at regular timer intervals."""
        if self.data_index >= len(self.csv_data):
            self.get_logger().info('End of CSV data reached.')
            return

        row = self.csv_data[self.data_index]
        yaw, pitch, roll = float(row[0]), float(row[1]), float(row[2])
        q = quaternion_from_euler(roll, pitch, yaw)

        quaternion_msg = Quaternion()
        quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w = q
        self.publisher.publish(quaternion_msg)
        self.data_index += 1
        self.get_logger().info(f'Publishing quaternion: {quaternion_msg}')

        # Broadcast the TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'cubesat'
        t.transform.translation.x = 0.0  # Adjust if your model has a position offset
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = quaternion_msg

        self.br.sendTransform(t)
        self.publish_marker(quaternion_msg=quaternion_msg)

    def publish_marker(self, quaternion_msg):
        marker = Marker()
        marker.header.frame_id = "cubesat"  # Use the child frame ID
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 1.0  # Size of the marker [m]
        marker.scale.y = 1.0
        marker.scale.z = 3.0
        marker.color.a = 1.0  # Alpha must be non-zero
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = quaternion_msg.w  # Neutral orientation
        marker.pose.orientation.x = quaternion_msg.x  # Position at the origin of the frame
        marker.pose.orientation.y = quaternion_msg.y
        marker.pose.orientation.z = quaternion_msg.z
        marker.pose.position.x = 0.0  # Optional: Adjust if the marker has a specific position
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = CSVReaderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


