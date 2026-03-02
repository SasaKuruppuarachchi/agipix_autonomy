#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import MultiThreadedExecutor
import tf2_ros

class OdometryToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path_node')
        print("pubing_start")
        
        self.pub_tf = False
        self.topic_out = '/Odometry'
        self.path_out = '/Odometry_path'
        self.parent_frame = 'camera_init'
        if self.pub_tf: self.new_frame = 'odom_base'
        
        self.do_path = True
        
        if not self.do_path:
            return
        
        self.subscription = self.create_subscription(
            Odometry,
            self.topic_out,
            self.odometry_callback,
            10)
        if self.pub_tf: self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        #for path
        self.path_publisher = self.create_publisher(Path, self.path_out, 10)
        self.path = Path()
        self.path.header.frame_id = self.parent_frame


    def odometry_callback(self, msg):
        tf = self.getpath_from_odom(msg)
        self.path_publisher.publish(self.path)
        if self.pub_tf: self.tf_broadcaster.sendTransform(tf)
        
    def getpath_from_odom(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        #pose.header.frame_id = self.parent_frame
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        
        if self.pub_tf:
            transform = TransformStamped()
            transform.header = msg.header
            #transform.header.frame_id = self.parent_frame
            transform.child_frame_id = self.new_frame
            transform.transform.translation.x = pose.pose.position.x
            transform.transform.translation.y = pose.pose.position.y
            transform.transform.translation.z = pose.pose.position.z
            transform.transform.rotation.w = pose.pose.orientation.w
            transform.transform.rotation.x = pose.pose.orientation.x
            transform.transform.rotation.y = pose.pose.orientation.y
            transform.transform.rotation.z = pose.pose.orientation.z
            return transform
        

def main(args=None):
    rclpy.init()
    odom_to_path = OdometryToPath()
    executor = MultiThreadedExecutor()
    executor.add_node(odom_to_path)
    try:
        executor.spin()
    except KeyboardInterrupt:
        odom_to_path.get_logger().info("Shutting down.")
        odom_to_path.on_shutdown()
    finally:
        odom_to_path.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    print("pubing_start")
    main()