import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import csv

class CSVReaderNode(Node):
    def __init__(self):
        super().__init__('csv_reader_node')
        self.publisher = self.create_publisher(Quaternion, 'orientation', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.csv_file_path = '/AERO740/rotation_visualization_ws/rotation_visualization/data/ADS.csv'
        self.csv_data = self.read_csv(self.csv_file_path)
        self.data_index = 0

    def read_csv(self, file_path):
        with open(file_path, newline='') as csvfile:
            data_reader = csv.reader(csvfile, delimiter=',')
            next(data_reader, None)  # Skip header
            return [row for row in data_reader]

    def timer_callback(self):
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
        self.get_logger().info(f'Publishing: {quaternion_msg}')

def main(args=None):
    rclpy.init(args=args)
    node = CSVReaderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

