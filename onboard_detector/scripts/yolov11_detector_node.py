#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from yolov11_detector import yolo_detector


def main():
	rclpy.init()
	node = yolo_detector()
	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
	
