#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data

class PX4IMUNode(Node):
    def __init__(self):
        super().__init__('px4_imu_node')
        self.namespace = 'drone'
        self.id = 0
        self.subscription = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.sensor_combined_callback,
            qos_profile=qos_profile_sensor_data)
        self.publisher = self.create_publisher(Imu, self.namespace + str(self.id) + '/px4_imu', 10)

    def sensor_combined_callback(self, msg):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'body'
        
        # Fill the IMU message with data from SensorCombined
        imu_msg.angular_velocity.x = float(msg.gyro_rad[0])
        imu_msg.angular_velocity.y = -float(msg.gyro_rad[1])
        imu_msg.angular_velocity.z = -float(msg.gyro_rad[2])
        
        imu_msg.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
        imu_msg.linear_acceleration.y = -float(msg.accelerometer_m_s2[1])
        imu_msg.linear_acceleration.z = -float(msg.accelerometer_m_s2[2])
        
        # Publish the IMU message
        self.publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PX4IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()