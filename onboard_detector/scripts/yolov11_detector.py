#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import torch
import os
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
# from sensor_msgs.msg import poin
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection2D
from cv_bridge import CvBridge
from ultralytics import YOLO

target_classes = ["person"]


path_curr = os.path.dirname(__file__)
img_topic = "/camera/color/image_raw"
device = "cuda" if torch.cuda.is_available() else "cpu"
weight = "weights/yolo11n.pt"
class_names = "config/coco.names"

class yolo_detector(Node):
    def __init__(self):
        super().__init__("yolov11_detector")
        self.get_logger().info("[onboardDetector]: yolo detector init...")

        self.img_received = False
        self.img_detected = False
        print(f"Using device: {device}")

        # init and load
        self.model = YOLO(os.path.join(path_curr, weight))
        # self.model = Detector(80, True).to(device)
        # self.model.load_state_dict(torch.load(os.path.join(path_curr, weight), map_location=device))
        # Some versions of ultralytics don't require explicit eval mode; keep if available
        try:
            self.model.eval()
        except Exception:
            pass

        # subscriber
        self.br = CvBridge()
        # Align ROS behavior to yolo_detector: parameterize color image topic
        self.declare_parameter('color_image_topic', '/color/preview/image') 
        color_image_topic = self.get_parameter('color_image_topic').get_parameter_value().string_value
        self.img_sub = self.create_subscription(Image, color_image_topic, self.image_callback, 10)
        self.get_logger().info(f"[onboardDetector]: YOLOv11 color image topic name: {color_image_topic}.")

        # publisher (match topics to yolo_detector with leading slash)
        self.img_pub = self.create_publisher(Image, "/yolo_detector/detected_image", 10)
        self.bbox_pub = self.create_publisher(Detection2DArray, "/yolo_detector/detected_bounding_boxes", 10)
        self.time_pub = self.create_publisher(Float64, "/yolo_detector/yolo_time", 10)

        # timer (parameterize like yolo_detector)
        self.declare_parameter('detect_timer_period', 0.033)
        time_step = self.get_parameter('detect_timer_period').value
        self.detect_timer = self.create_timer(time_step, self.detect_callback)
        self.get_logger().info(f"[onboardDetector]: Time step is set to: {time_step}s")
        self.bbox_timer = self.create_timer(time_step, self.bbox_callback)

        self.declare_parameter('debug_visualization', False)
        debug_vis = self.get_parameter('debug_visualization').value
        if debug_vis:
            self.vis_timer = self.create_timer(time_step, self.vis_callback)
        self.get_logger().info(f"[onboardDetector]: Debug visualization is set to: {debug_vis}.")
    
    def image_callback(self, msg):
        self.img = self.br.imgmsg_to_cv2(msg, "bgr8")
        self.img_received = True

    def detect_callback(self):
        startTime = self.get_clock().now()
        if (self.img_received == True):
            output = self.inference(self.img)
            self.detected_img, self.detected_bboxes = self.postprocess(self.img, output)
            self.img_detected = True
        endTime = self.get_clock().now()
        elapsed = (endTime - startTime).nanoseconds / 1e9
        self.time_pub.publish(Float64(data=float(elapsed)))
        

    def vis_callback(self):
        if (self.img_detected == True):
            self.img_pub.publish(self.br.cv2_to_imgmsg(self.detected_img, "bgr8"))

    def bbox_callback(self):
        if (self.img_detected == True):
            bboxes_msg = Detection2DArray()
            for detected_box in self.detected_bboxes:
                if (detected_box[4] in target_classes):
                    bbox_msg = Detection2D()
                    bbox_msg.bbox.center.position.x = float(int(detected_box[0]))
                    bbox_msg.bbox.center.position.y = float(int(detected_box[1]))
                    bbox_msg.bbox.size_x = float(abs(detected_box[2] - detected_box[0])) 
                    bbox_msg.bbox.size_y = float(abs(detected_box[3] - detected_box[1]))

                    bboxes_msg.detections.append(bbox_msg)
                bboxes_msg.header.stamp = self.get_clock().now().to_msg()
            self.bbox_pub.publish(bboxes_msg)

    def inference(self, ori_img):
        # image pre-processing
        res_img = cv2.resize(ori_img, (352, 352), interpolation = cv2.INTER_LINEAR) 
        img = res_img.reshape(1, 352, 352, 3)
        img = torch.from_numpy(img.transpose(0, 3, 1, 2))
        img = img.to(device).float() / 255.0   

        # inference
        preds = self.model(img, device=device, half=True, verbose=False)[0]
        return [preds.boxes.xyxyn, preds.boxes.conf, preds.boxes.cls]

    def postprocess(self, ori_img, output):
        LABEL_NAMES = []
        with open(os.path.join(path_curr, class_names), 'r') as f:
            for line in f.readlines():
                LABEL_NAMES.append(line.strip())
        
        H, W, _ = ori_img.shape

        detected_boxes = []
        for i, box in enumerate(output[0]):
            box = box.tolist()
           
            obj_score = output[1][i]
            category = LABEL_NAMES[int(output[2][i])]
            x1, y1 = int(box[0] * W), int(box[1] * H)
            x2, y2 = int(box[2] * W), int(box[3] * H)
            detected_box = [x1, y1, x2, y2, category]
            detected_boxes.append(detected_box)

            cv2.rectangle(ori_img, (x1, y1), (x2, y2), (255, 255, 0), 2)
            cv2.putText(ori_img, '%.2f' % obj_score, (x1, y1 - 5), 0, 0.7, (0, 255, 0), 2)  
            cv2.putText(ori_img, category, (x1, y1 - 25), 0, 0.7, (0, 255, 0), 2)
        return ori_img, detected_boxes
