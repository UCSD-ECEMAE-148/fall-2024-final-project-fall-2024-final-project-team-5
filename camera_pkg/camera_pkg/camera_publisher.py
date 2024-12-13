#!/usr/bin/env python
from roboflowoak import RoboflowOak
import cv2
import time
import numpy as np
import rclpy
import math
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile

class target_publisher(Node):
    def __init__(self, node_name="target_publisher"):
        self._node_name = node_name
        self.angle_per_pix = 40.35/1080 #40.35 33.582
        self.center_x = 384
        
        advanced_config = {
        	"nn_mode": "device"
        }
        #face-detection-mik1i
        # self.rf = RoboflowOak(model="detect-car-jsvsk", confidence=0.80, overlap=0.5,
        #     version="4", api_key="7CfAEQ0li4t5GeTTqLXK", rgb=True,
        #     depth=True, device=None, blocking=True)
        self.rf = RoboflowOak(model="detect-car-jsvsk", confidence=0.70,
            overlap=0.5, version="4", api_key="7CfAEQ0li4t5GeTTqLXK", rgb=True,
            depth=False, device=None, device_name="", blocking=True, advanced_config=advanced_config)


        super().__init__(self._node_name)
        
        self.publisher_ = self.create_publisher(Float32MultiArray, '/camera_data', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.publish_target)

        self.get_logger().info(self._node_name +" Ready...")


    def publish_target(self):
        msg = Float32MultiArray()
        data = []
        result, frame, raw_frame, depth = self.rf.detect()
        predictions = result["predictions"]
        if len(predictions) == 0:
            msg.data = [float('inf'), float('inf')]
            self.publisher_.publish(msg)
            self.get_logger().info("no target")
            return
        confidence = 0
        best = predictions[0]
        #self.get_logger().info(f'num tar: %d' % len(predictions))
        for p in predictions:
            if p.confidence > confidence:
                confidence = p.confidence
                best = p
        x = best.x
        y = best.y
        #self.get_logger().info(f'shape: %d' % np.shape(frame)[1])
        # frame_height, frame_width = frame.shape[:2]
        # depth_resized = cv2.resize(depth, (frame_width, frame_height), interpolation=cv2.INTER_NEAREST)
        data.append(0.0)
        #calculate angle here
        data.append((self.center_x-x) * self.angle_per_pix)
        self.get_logger().info(f'angle: %d' % data[-1])
        msg.data = data
        #msg.data = [float('inf'), float('inf')]
        self.publisher_.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = target_publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

